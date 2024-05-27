/**@file        ppp_integ_dt_model_scene.c
 * @brief       PPP Scene Type Prediction Decision Tree Model Created by PYTHON
 * @details
 * @author      xiayi
 * @date        2024/1/11
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/1/11   <td>0.1      <td>xiayi    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "ppp_integ_model.h"

static uint8_t predict_scene_0(integ_PppFeature_t* gpz_pppFeature) {
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  uint8_t u_ppp_use_cp_num_float = gpz_pppFeature->u_ppp_use_cp_num_float;
  uint16_t w_cn0Num = gpz_pppFeature->w_cn0Num;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_posLlaUnc_b = gpz_pppFeature->f_posLlaUnc[0];
  uint8_t pu_CN0greater35NumPerSys_C = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_BDS3];
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t pu_noLLIsatNum_C = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_BDS3];
  float f_avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t pu_noLLIsatNum_G = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_GPS];
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  int32_t q_QRcheckStatus = (int32_t)gpz_pppFeature->q_QRcheckStatus;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  float f_posLlaUnc_h = gpz_pppFeature->f_posLlaUnc[2];
  float f_posLlaUnc_l = gpz_pppFeature->f_posLlaUnc[1];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  uint8_t pu_satNumPerSys_G = gpz_pppFeature->pu_satNumPerSys[C_GNSS_GPS];
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  uint16_t w_LLInum = gpz_pppFeature->w_LLInum;
  uint8_t pu_LLIsatNum_G = gpz_pppFeature->pu_LLIsatNum[C_GNSS_GPS];
  uint8_t pu_LLIsatNum_C = gpz_pppFeature->pu_LLIsatNum[C_GNSS_BDS3];
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  uint8_t pu_CN0greater35NumPerSys_G = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_GPS];
  uint8_t pu_satNumPerSys_C = gpz_pppFeature->pu_satNumPerSys[C_GNSS_BDS3];
  float f_pdop = gpz_pppFeature->z_dops.f_pdop;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;
  float f_max_CN0 = gpz_pppFeature->f_max_CN0;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  uint16_t w_noLLInum = gpz_pppFeature->w_noLLInum;

  if (f_postCodeSTD <= 1.1) {
    if (f_posLlaUnc_h <= 0.79) {
      if (u_CN040 <= 8.5) {
        if (w_noLLInum <= 18.5) {
          if (f_cmc_avg <= 3.29) {
            if (f_posLlaUnc_h <= 0.62) {
              if (pu_noLLIsatNum_G <= 4.5) {
                if (f_posLlaUnc_l <= 0.39) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.39
                  return 1;
                }
              }
              else { // pu_noLLIsatNum_G > 4.5
                if (pl_ekf_ver <= 0.47) {
                  return 1;
                }
                else { // pl_ekf_ver > 0.47
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.62
              if (pu_satNumPerSys_C <= 22.5) {
                if (pl_ekf_hor <= 2.69) {
                  return 1;
                }
                else { // pl_ekf_hor > 2.69
                  return 3;
                }
              }
              else { // pu_satNumPerSys_C > 22.5
                return 3;
              }
            }
          }
          else { // f_cmc_avg > 3.29
            if (f_gdop <= 3.95) {
              if (f_max_CN0 <= 41.5) {
                return 1;
              }
              else { // f_max_CN0 > 41.5
                if (u_satMeasCount <= 23.5) {
                  return 3;
                }
                else { // u_satMeasCount > 23.5
                  return 1;
                }
              }
            }
            else { // f_gdop > 3.95
              return 1;
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (pu_CN0greater35NumPerSys_C <= 6.5) {
            if (f_posLlaUnc_b <= 0.11) {
              if (f_vdop <= 1.14) {
                if (f_cmc_avg <= 1.68) {
                  return 1;
                }
                else { // f_cmc_avg > 1.68
                  return 3;
                }
              }
              else { // f_vdop > 1.14
                if (u_MeasInUseCount <= 30.5) {
                  return 3;
                }
                else { // u_MeasInUseCount > 30.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.11
              if (pu_LLIsatNum_C <= 3.5) {
                if (pu_noLLIsatNum_C <= 11.5) {
                  return 3;
                }
                else { // pu_noLLIsatNum_C > 11.5
                  return 4;
                }
              }
              else { // pu_LLIsatNum_C > 3.5
                if (pu_satNumPerSys_G <= 7.5) {
                  return 2;
                }
                else { // pu_satNumPerSys_G > 7.5
                  return 1;
                }
              }
            }
          }
          else { // pu_CN0greater35NumPerSys_C > 6.5
            if (f_max_CN0 <= 49.5) {
              if (f_postPhaseSTD <= 0.03) {
                if (f_avg_unc_pr <= 3.95) {
                  return 1;
                }
                else { // f_avg_unc_pr > 3.95
                  return 1;
                }
              }
              else { // f_postPhaseSTD > 0.03
                return 2;
              }
            }
            else { // f_max_CN0 > 49.5
              if (f_posLlaUnc_b <= 0.11) {
                if (f_cmc_avg <= 2.71) {
                  return 3;
                }
                else { // f_cmc_avg > 2.71
                  return 2;
                }
              }
              else { // f_posLlaUnc_b > 0.11
                if (f_posLlaUnc_b <= 0.14) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.14
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 8.5
        if (f_postCodeSTD <= 0.74) {
          if (u_ppp_use_cp_num_float <= 41.5) {
            if (pl_ekf_ver <= 0.2) {
              if (f_postCodeSTD <= 0.41) {
                if (f_posLlaUnc_h <= 0.04) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.04
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.41
                if (w_cn0Num <= 23.5) {
                  return 2;
                }
                else { // w_cn0Num > 23.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.2
              if (f_cmc_std <= 2.57) {
                if (f_max_CN0 <= 51.5) {
                  return 1;
                }
                else { // f_max_CN0 > 51.5
                  return 3;
                }
              }
              else { // f_cmc_std > 2.57
                if (f_avg_unc_pr <= 12.95) {
                  return 1;
                }
                else { // f_avg_unc_pr > 12.95
                  return 1;
                }
              }
            }
          }
          else { // u_ppp_use_cp_num_float > 41.5
            if (f_postCodeSTD <= 0.53) {
              if (pu_CN0greater35NumPerSys_C <= 19.5) {
                if (u_ppp_use_pr_num_float <= 35.5) {
                  return 1;
                }
                else { // u_ppp_use_pr_num_float > 35.5
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 19.5
                if (f_vdop <= 1.0) {
                  return 1;
                }
                else { // f_vdop > 1.0
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.53
              if (f_hdop <= 0.63) {
                if (w_noLLInum <= 36.5) {
                  return 1;
                }
                else { // w_noLLInum > 36.5
                  return 1;
                }
              }
              else { // f_hdop > 0.63
                if (pu_CN0greater35NumPerSys_G <= 8.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 8.5
                  return 1;
                }
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.74
          if (f_cn0_std <= 7.78) {
            if (pl_ekf_hor <= 1.42) {
              if (w_noLLInum <= 15.5) {
                if (pl_ekf_ver <= 0.11) {
                  return 1;
                }
                else { // pl_ekf_ver > 0.11
                  return 1;
                }
              }
              else { // w_noLLInum > 15.5
                if (pu_CN0greater35NumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 7.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 1.42
              if (f_postCodeSTD <= 0.92) {
                if (pu_CN0greater35NumPerSys_C <= 13.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 13.5
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.92
                if (pu_satNumPerSys_C <= 12.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 12.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_std > 7.78
            if (pl_ekf_hor <= 0.55) {
              if (u_CN040 <= 12.5) {
                if (f_posLlaUnc_b <= 0.03) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.03
                  return 2;
                }
              }
              else { // u_CN040 > 12.5
                if (f_posLlaUnc_l <= 0.03) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.03
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 0.55
              if (w_LLInum <= 7.5) {
                if (f_cmc_avg <= 0.65) {
                  return 2;
                }
                else { // f_cmc_avg > 0.65
                  return 1;
                }
              }
              else { // w_LLInum > 7.5
                if (f_posLlaUnc_h <= 0.65) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.65
                  return 1;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.79
      if (f_posLlaUnc_l <= 0.94) {
        if (w_noLLInum <= 4.5) {
          if (f_gdop <= 2.97) {
            if (pu_CN0greater35NumPerSys_C <= 11.5) {
              if (u_MeasInUseCount <= 34.5) {
                if (pu_CN0greater35NumPerSys_G <= 2.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 2.5
                  return 3;
                }
              }
              else { // u_MeasInUseCount > 34.5
                if (u_CN040 <= 7.5) {
                  return 3;
                }
                else { // u_CN040 > 7.5
                  return 1;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 11.5
              if (f_postCodeSTD <= 0.9) {
                if (pl_ekf_ver <= 3.9) {
                  return 1;
                }
                else { // pl_ekf_ver > 3.9
                  return 3;
                }
              }
              else { // f_postCodeSTD > 0.9
                if (f_posLlaUnc_l <= 0.25) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.25
                  return 1;
                }
              }
            }
          }
          else { // f_gdop > 2.97
            if (pu_CN0greater35NumPerSys_C <= 4.5) {
              if (w_cn0Num <= 31.5) {
                if (w_noLLInum <= 0.5) {
                  return 4;
                }
                else { // w_noLLInum > 0.5
                  return 1;
                }
              }
              else { // w_cn0Num > 31.5
                if (f_postPhaseSTD <= 0.0) {
                  return 4;
                }
                else { // f_postPhaseSTD > 0.0
                  return 3;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 4.5
              if (pu_CN0greater35NumPerSys_C <= 10.5) {
                if (f_posLlaUnc_b <= 0.51) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.51
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 10.5
                if (f_cmc_std <= 1.14) {
                  return 3;
                }
                else { // f_cmc_std > 1.14
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 4.5
          if (f_avg_unc_pr <= 10.42) {
            if (f_postCodeSTD <= 0.67) {
              if (f_cn0_avg <= 29.75) {
                if (f_cn0_std <= 7.78) {
                  return 4;
                }
                else { // f_cn0_std > 7.78
                  return 4;
                }
              }
              else { // f_cn0_avg > 29.75
                if (pl_ekf_hor <= 1.01) {
                  return 3;
                }
                else { // pl_ekf_hor > 1.01
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.67
              if (pu_CN0greater35NumPerSys_C <= 12.5) {
                if (pu_noLLIsatNum_G <= 5.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_G > 5.5
                  return 3;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 12.5
                if (f_postCodeSTD <= 0.98) {
                  return 1;
                }
                else { // f_postCodeSTD > 0.98
                  return 1;
                }
              }
            }
          }
          else { // f_avg_unc_pr > 10.42
            if (f_postPhaseSTD <= 0.01) {
              if (pu_satNumPerSys_C <= 18.5) {
                if (pu_noLLIsatNum_G <= 6.5) {
                  return 4;
                }
                else { // pu_noLLIsatNum_G > 6.5
                  return 3;
                }
              }
              else { // pu_satNumPerSys_C > 18.5
                if (f_posLlaUnc_l <= 0.26) {
                  return 3;
                }
                else { // f_posLlaUnc_l > 0.26
                  return 1;
                }
              }
            }
            else { // f_postPhaseSTD > 0.01
              if (f_posLlaUnc_b <= 0.45) {
                if (pu_CN0greater35NumPerSys_C <= 5.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 5.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.45
                if (pu_satNumPerSys_C <= 16.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 16.5
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // f_posLlaUnc_l > 0.94
        if (u_ppp_use_pr_num_float <= 9.5) {
          if (u_MeasTrackCount <= 36.5) {
            if (f_posLlaUnc_h <= 2.48) {
              if (f_cn0_avg <= 31.5) {
                if (u_satMeasCount <= 13.5) {
                  return 4;
                }
                else { // u_satMeasCount > 13.5
                  return 3;
                }
              }
              else { // f_cn0_avg > 31.5
                if (u_satMeasCount <= 17.5) {
                  return 1;
                }
                else { // u_satMeasCount > 17.5
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 2.48
              if (f_cn0_avg <= 44.75) {
                if (pu_LLIsatNum_C <= 13.5) {
                  return 4;
                }
                else { // pu_LLIsatNum_C > 13.5
                  return 3;
                }
              }
              else { // f_cn0_avg > 44.75
                if (f_posLlaUnc_l <= 3.42) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 3.42
                  return 3;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 36.5
            if (pu_LLIsatNum_C <= 14.5) {
              if (pu_LLIsatNum_C <= 11.5) {
                if (pu_noLLIsatNum_G <= 4.5) {
                  return 4;
                }
                else { // pu_noLLIsatNum_G > 4.5
                  return 1;
                }
              }
              else { // pu_LLIsatNum_C > 11.5
                if (f_posLlaUnc_h <= 1.98) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 1.98
                  return 4;
                }
              }
            }
            else { // pu_LLIsatNum_C > 14.5
              if (f_cmc_std <= 0.6) {
                return 3;
              }
              else { // f_cmc_std > 0.6
                if (f_posLlaUnc_h <= 1.8) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.8
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 9.5
          if (f_cn0_avg <= 36.25) {
            if (f_posLlaUnc_b <= 0.72) {
              if (f_posLlaUnc_l <= 1.69) {
                if (u_MeasTrackCount <= 34.0) {
                  return 1;
                }
                else { // u_MeasTrackCount > 34.0
                  return 3;
                }
              }
              else { // f_posLlaUnc_l > 1.69
                return 4;
              }
            }
            else { // f_posLlaUnc_b > 0.72
              if (pu_LLIsatNum_C <= 13.5) {
                if (f_postCodeSTD <= 1.02) {
                  return 4;
                }
                else { // f_postCodeSTD > 1.02
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 13.5
                return 1;
              }
            }
          }
          else { // f_cn0_avg > 36.25
            if (u_MeasTrackCount <= 24.5) {
              if (u_satMeasCount <= 14.5) {
                if (w_noLLInum <= 3.5) {
                  return 3;
                }
                else { // w_noLLInum > 3.5
                  return 1;
                }
              }
              else { // u_satMeasCount > 14.5
                if (f_hdop <= 1.31) {
                  return 3;
                }
                else { // f_hdop > 1.31
                  return 4;
                }
              }
            }
            else { // u_MeasTrackCount > 24.5
              if (f_hdop <= 0.82) {
                if (f_avg_unc_pr <= 5.32) {
                  return 3;
                }
                else { // f_avg_unc_pr > 5.32
                  return 1;
                }
              }
              else { // f_hdop > 0.82
                if (f_cmc_avg <= 3.86) {
                  return 1;
                }
                else { // f_cmc_avg > 3.86
                  return 3;
                }
              }
            }
          }
        }
      }
    }
  }
  else { // f_postCodeSTD > 1.1
    if (f_posLlaUnc_h <= 0.67) {
      if (u_CN040 <= 7.5) {
        if (w_noLLInum <= 16.5) {
          if (u_satMeasCount <= 24.5) {
            if (f_cmc_avg <= 3.0) {
              if (pl_ekf_hor <= 1.26) {
                if (f_max_CN0 <= 49.5) {
                  return 1;
                }
                else { // f_max_CN0 > 49.5
                  return 2;
                }
              }
              else { // pl_ekf_hor > 1.26
                if (f_posLlaUnc_b <= 0.34) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.34
                  return 3;
                }
              }
            }
            else { // f_cmc_avg > 3.0
              if (f_cn0_avg <= 28.75) {
                if (pu_LLIsatNum_C <= 14.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_C > 14.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 28.75
                if (f_posLlaUnc_h <= 0.57) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.57
                  return 3;
                }
              }
            }
          }
          else { // u_satMeasCount > 24.5
            if (w_noLLInum <= 6.5) {
              if (f_posLlaUnc_h <= 0.53) {
                if (f_posLlaUnc_b <= 0.21) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.21
                  return 1;
                }
              }
              else { // f_posLlaUnc_h > 0.53
                if (pu_noLLIsatNum_C <= 2.5) {
                  return 3;
                }
                else { // pu_noLLIsatNum_C > 2.5
                  return 1;
                }
              }
            }
            else { // w_noLLInum > 6.5
              if (f_posLlaUnc_l <= 0.13) {
                if (q_QRcheckStatus <= 0.5) {
                  return 2;
                }
                else { // q_QRcheckStatus > 0.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.13
                if (pu_LLIsatNum_G <= 7.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_G > 7.5
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 16.5
          if (pu_noLLIsatNum_C <= 13.5) {
            if (f_posLlaUnc_h <= 0.43) {
              if (f_vdop <= 1.17) {
                if (u_ppp_use_cp_num_float <= 25.5) {
                  return 1;
                }
                else { // u_ppp_use_cp_num_float > 25.5
                  return 3;
                }
              }
              else { // f_vdop > 1.17
                if (f_max_CN0 <= 41.5) {
                  return 1;
                }
                else { // f_max_CN0 > 41.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.43
              if (f_posLlaUnc_b <= 0.16) {
                if (w_LLInum <= 18.0) {
                  return 1;
                }
                else { // w_LLInum > 18.0
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 6.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 6.5
                  return 1;
                }
              }
            }
          }
          else { // pu_noLLIsatNum_C > 13.5
            if (f_posLlaUnc_b <= 0.12) {
              if (pu_CN0greater35NumPerSys_C <= 7.5) {
                if (u_CN040 <= 6.5) {
                  return 3;
                }
                else { // u_CN040 > 6.5
                  return 2;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 7.5
                if (w_cn0Num <= 36.5) {
                  return 2;
                }
                else { // w_cn0Num > 36.5
                  return 2;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.12
              if (u_MeasTrackCount <= 55.5) {
                if (pu_satNumPerSys_G <= 7.5) {
                  return 4;
                }
                else { // pu_satNumPerSys_G > 7.5
                  return 3;
                }
              }
              else { // u_MeasTrackCount > 55.5
                if (f_cn0_avg <= 30.5) {
                  return 3;
                }
                else { // f_cn0_avg > 30.5
                  return 1;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 7.5
        if (w_noLLInum <= 18.5) {
          if (pl_ekf_hor <= 1.24) {
            if (f_cmc_std <= 2.07) {
              if (f_cn0_avg <= 43.25) {
                if (w_LLInum <= 21.5) {
                  return 1;
                }
                else { // w_LLInum > 21.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 43.25
                if (f_posLlaUnc_h <= 0.42) {
                  return 4;
                }
                else { // f_posLlaUnc_h > 0.42
                  return 3;
                }
              }
            }
            else { // f_cmc_std > 2.07
              if (pl_ekf_hor <= 0.78) {
                if (f_cmc_avg <= 6.2) {
                  return 1;
                }
                else { // f_cmc_avg > 6.2
                  return 2;
                }
              }
              else { // pl_ekf_hor > 0.78
                if (u_ppp_use_cp_num_float <= 41.5) {
                  return 1;
                }
                else { // u_ppp_use_cp_num_float > 41.5
                  return 3;
                }
              }
            }
          }
          else { // pl_ekf_hor > 1.24
            if (q_QRcheckStatus <= 0.5) {
              if (f_cn0_avg <= 29.25) {
                if (f_posLlaUnc_l <= 0.23) {
                  return 3;
                }
                else { // f_posLlaUnc_l > 0.23
                  return 1;
                }
              }
              else { // f_cn0_avg > 29.25
                if (w_noLLInum <= 11.5) {
                  return 1;
                }
                else { // w_noLLInum > 11.5
                  return 1;
                }
              }
            }
            else { // q_QRcheckStatus > 0.5
              if (pl_ekf_hor <= 1.51) {
                if (pl_ekf_ver <= 0.65) {
                  return 3;
                }
                else { // pl_ekf_ver > 0.65
                  return 1;
                }
              }
              else { // pl_ekf_hor > 1.51
                if (pu_satNumPerSys_G <= 8.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_G > 8.5
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (f_cn0_avg <= 38.25) {
            if (f_posLlaUnc_h <= 0.18) {
              if (f_cmc_avg <= 3.88) {
                if (f_postPhaseSTD <= 0.01) {
                  return 2;
                }
                else { // f_postPhaseSTD > 0.01
                  return 3;
                }
              }
              else { // f_cmc_avg > 3.88
                if (f_postPhaseSTD <= 0.01) {
                  return 3;
                }
                else { // f_postPhaseSTD > 0.01
                  return 2;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.18
              if (f_posLlaUnc_b <= 0.16) {
                if (f_postCodeSTD <= 2.8) {
                  return 1;
                }
                else { // f_postCodeSTD > 2.8
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 8.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 8.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_avg > 38.25
            if (f_postCodeSTD <= 1.75) {
              if (f_posLlaUnc_b <= 0.15) {
                if (f_gdop <= 1.2) {
                  return 2;
                }
                else { // f_gdop > 1.2
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.15
                if (u_ppp_use_pr_num_float <= 16.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 16.5
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 1.75
              if (f_posLlaUnc_b <= 0.03) {
                if (u_MeasTrackCount <= 62.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 62.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.03
                if (f_pdop <= 1.22) {
                  return 2;
                }
                else { // f_pdop > 1.22
                  return 1;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.67
      if (pl_ekf_hor <= 3.15) {
        if (f_cn0_avg <= 30.75) {
          if (pu_satNumPerSys_C <= 14.5) {
            if (pl_ekf_hor <= 2.31) {
              if (u_satMeasCount <= 16.0) {
                if (f_posLlaUnc_h <= 0.69) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.69
                  return 1;
                }
              }
              else { // u_satMeasCount > 16.0
                if (f_posLlaUnc_b <= 0.38) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.38
                  return 3;
                }
              }
            }
            else { // pl_ekf_hor > 2.31
              if (pl_ekf_ver <= 1.6) {
                if (pl_ekf_hor <= 2.74) {
                  return 3;
                }
                else { // pl_ekf_hor > 2.74
                  return 3;
                }
              }
              else { // pl_ekf_ver > 1.6
                if (f_max_CN0 <= 46.5) {
                  return 3;
                }
                else { // f_max_CN0 > 46.5
                  return 4;
                }
              }
            }
          }
          else { // pu_satNumPerSys_C > 14.5
            if (f_max_CN0 <= 46.5) {
              if (pl_ekf_ver <= 1.42) {
                if (pu_LLIsatNum_C <= 17.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_C > 17.5
                  return 1;
                }
              }
              else { // pl_ekf_ver > 1.42
                if (w_noLLInum <= 14.5) {
                  return 3;
                }
                else { // w_noLLInum > 14.5
                  return 1;
                }
              }
            }
            else { // f_max_CN0 > 46.5
              if (u_MeasInUseCount <= 29.5) {
                if (f_posLlaUnc_h <= 1.35) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.35
                  return 1;
                }
              }
              else { // u_MeasInUseCount > 29.5
                if (f_max_CN0 <= 48.5) {
                  return 1;
                }
                else { // f_max_CN0 > 48.5
                  return 4;
                }
              }
            }
          }
        }
        else { // f_cn0_avg > 30.75
          if (pl_ekf_ver <= 1.26) {
            if (u_MeasInUseCount <= 42.5) {
              if (f_max_CN0 <= 48.5) {
                if (f_cn0_std <= 5.56) {
                  return 3;
                }
                else { // f_cn0_std > 5.56
                  return 1;
                }
              }
              else { // f_max_CN0 > 48.5
                if (u_CN040 <= 20.5) {
                  return 1;
                }
                else { // u_CN040 > 20.5
                  return 3;
                }
              }
            }
            else { // u_MeasInUseCount > 42.5
              if (f_posLlaUnc_h <= 0.84) {
                if (u_ppp_use_cp_num_float <= 36.5) {
                  return 3;
                }
                else { // u_ppp_use_cp_num_float > 36.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_h > 0.84
                if (f_posLlaUnc_b <= 0.29) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.29
                  return 3;
                }
              }
            }
          }
          else { // pl_ekf_ver > 1.26
            if (w_cn0Num <= 37.5) {
              if (f_cn0_avg <= 37.25) {
                if (w_LLInum <= 17.5) {
                  return 4;
                }
                else { // w_LLInum > 17.5
                  return 3;
                }
              }
              else { // f_cn0_avg > 37.25
                if (w_noLLInum <= 19.5) {
                  return 3;
                }
                else { // w_noLLInum > 19.5
                  return 3;
                }
              }
            }
            else { // w_cn0Num > 37.5
              if (pl_ekf_hor <= 2.26) {
                if (f_cmc_std <= 0.42) {
                  return 3;
                }
                else { // f_cmc_std > 0.42
                  return 3;
                }
              }
              else { // pl_ekf_hor > 2.26
                if (f_postCodeSTD <= 1.76) {
                  return 3;
                }
                else { // f_postCodeSTD > 1.76
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 3.15
        if (u_ppp_use_pr_num_float <= 12.5) {
          if (w_LLInum <= 12.5) {
            if (pl_ekf_hor <= 4.24) {
              if (f_cn0_std <= 8.53) {
                if (pl_ekf_ver <= 1.93) {
                  return 4;
                }
                else { // pl_ekf_ver > 1.93
                  return 3;
                }
              }
              else { // f_cn0_std > 8.53
                if (f_posLlaUnc_b <= 0.57) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.57
                  return 4;
                }
              }
            }
            else { // pl_ekf_hor > 4.24
              if (f_hdop <= 7.21) {
                if (f_max_CN0 <= 35.5) {
                  return 1;
                }
                else { // f_max_CN0 > 35.5
                  return 4;
                }
              }
              else { // f_hdop > 7.21
                if (f_posLlaUnc_b <= 1.35) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 1.35
                  return 4;
                }
              }
            }
          }
          else { // w_LLInum > 12.5
            if (f_max_CN0 <= 40.5) {
              if (pu_satNumPerSys_G <= 5.5) {
                if (f_posLlaUnc_h <= 1.8) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.8
                  return 4;
                }
              }
              else { // pu_satNumPerSys_G > 5.5
                if (f_posLlaUnc_l <= 0.72) {
                  return 4;
                }
                else { // f_posLlaUnc_l > 0.72
                  return 4;
                }
              }
            }
            else { // f_max_CN0 > 40.5
              if (f_posLlaUnc_l <= 0.39) {
                if (f_posLlaUnc_h <= 0.74) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.74
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.39
                if (f_posLlaUnc_b <= 0.36) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.36
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 12.5
          if (u_MeasTrackCount <= 35.5) {
            if (f_posLlaUnc_h <= 1.58) {
              if (pl_ekf_ver <= 2.83) {
                if (f_postCodeSTD <= 1.37) {
                  return 1;
                }
                else { // f_postCodeSTD > 1.37
                  return 3;
                }
              }
              else { // pl_ekf_ver > 2.83
                return 1;
              }
            }
            else { // f_posLlaUnc_h > 1.58
              if (u_MeasTrackCount <= 28.5) {
                if (f_posLlaUnc_l <= 1.01) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 1.01
                  return 1;
                }
              }
              else { // u_MeasTrackCount > 28.5
                if (pu_satNumPerSys_C <= 13.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 13.5
                  return 1;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 35.5
            if (f_posLlaUnc_h <= 1.37) {
              if (f_posLlaUnc_h <= 1.08) {
                if (f_cn0_avg <= 40.5) {
                  return 1;
                }
                else { // f_cn0_avg > 40.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_h > 1.08
                if (pu_CN0greater35NumPerSys_C <= 12.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 12.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 1.37
              if (f_cn0_avg <= 38.75) {
                if (f_posLlaUnc_l <= 0.38) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.38
                  return 4;
                }
              }
              else { // f_cn0_avg > 38.75
                if (u_ppp_use_pr_num_float <= 15.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 15.5
                  return 1;
                }
              }
            }
          }
        }
      }
    }
  }
}

static uint8_t predict_scene_1(integ_PppFeature_t* gpz_pppFeature) {
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  uint8_t u_ppp_use_cp_num_float = gpz_pppFeature->u_ppp_use_cp_num_float;
  uint16_t w_cn0Num = gpz_pppFeature->w_cn0Num;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_posLlaUnc_b = gpz_pppFeature->f_posLlaUnc[0];
  uint8_t pu_CN0greater35NumPerSys_C = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_BDS3];
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  float f_avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t pu_noLLIsatNum_C = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_BDS3];
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t pu_noLLIsatNum_G = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_GPS];
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  int32_t q_QRcheckStatus = (int32_t)gpz_pppFeature->q_QRcheckStatus;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  float f_posLlaUnc_h = gpz_pppFeature->f_posLlaUnc[2];
  float f_posLlaUnc_l = gpz_pppFeature->f_posLlaUnc[1];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  uint8_t pu_satNumPerSys_G = gpz_pppFeature->pu_satNumPerSys[C_GNSS_GPS];
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  uint16_t w_LLInum = gpz_pppFeature->w_LLInum;
  uint8_t pu_LLIsatNum_C = gpz_pppFeature->pu_LLIsatNum[C_GNSS_BDS3];
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  uint8_t pu_CN0greater35NumPerSys_G = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_GPS];
  uint8_t pu_satNumPerSys_C = gpz_pppFeature->pu_satNumPerSys[C_GNSS_BDS3];
  float f_pdop = gpz_pppFeature->z_dops.f_pdop;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;
  float f_max_CN0 = gpz_pppFeature->f_max_CN0;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  uint16_t w_noLLInum = gpz_pppFeature->w_noLLInum;

  if (f_postCodeSTD <= 1.12) {
    if (f_posLlaUnc_h <= 0.79) {
      if (u_CN040 <= 8.5) {
        if (w_noLLInum <= 18.5) {
          if (f_cmc_avg <= 3.29) {
            if (f_posLlaUnc_h <= 0.63) {
              if (f_posLlaUnc_l <= 0.39) {
                if (f_max_CN0 <= 48.5) {
                  return 1;
                }
                else { // f_max_CN0 > 48.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.39
                if (f_posLlaUnc_b <= 0.19) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.19
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.63
              if (pu_satNumPerSys_C <= 22.5) {
                if (pl_ekf_hor <= 2.69) {
                  return 1;
                }
                else { // pl_ekf_hor > 2.69
                  return 3;
                }
              }
              else { // pu_satNumPerSys_C > 22.5
                return 3;
              }
            }
          }
          else { // f_cmc_avg > 3.29
            if (f_gdop <= 3.95) {
              if (f_max_CN0 <= 43.5) {
                return 1;
              }
              else { // f_max_CN0 > 43.5
                if (u_MeasTrackCount <= 51.0) {
                  return 3;
                }
                else { // u_MeasTrackCount > 51.0
                  return 3;
                }
              }
            }
            else { // f_gdop > 3.95
              return 1;
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (f_posLlaUnc_h <= 0.29) {
            if (pu_CN0greater35NumPerSys_C <= 6.5) {
              if (f_posLlaUnc_b <= 0.12) {
                if (f_vdop <= 1.12) {
                  return 1;
                }
                else { // f_vdop > 1.12
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.12
                return 4;
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 6.5
              if (f_posLlaUnc_b <= 0.11) {
                if (f_cmc_avg <= 2.71) {
                  return 1;
                }
                else { // f_cmc_avg > 2.71
                  return 2;
                }
              }
              else { // f_posLlaUnc_b > 0.11
                if (f_vdop <= 1.5) {
                  return 4;
                }
                else { // f_vdop > 1.5
                  return 3;
                }
              }
            }
          }
          else { // f_posLlaUnc_h > 0.29
            if (f_max_CN0 <= 49.5) {
              if (u_MeasInUseCount <= 25.5) {
                return 3;
              }
              else { // u_MeasInUseCount > 25.5
                if (f_posLlaUnc_h <= 0.34) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.34
                  return 1;
                }
              }
            }
            else { // f_max_CN0 > 49.5
              if (f_cmc_avg <= 2.48) {
                if (f_postCodeSTD <= 0.66) {
                  return 3;
                }
                else { // f_postCodeSTD > 0.66
                  return 4;
                }
              }
              else { // f_cmc_avg > 2.48
                if (u_ppp_use_pr_num_float <= 13.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 13.5
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 8.5
        if (f_postCodeSTD <= 0.74) {
          if (u_ppp_use_cp_num_float <= 41.5) {
            if (pl_ekf_ver <= 0.2) {
              if (f_postCodeSTD <= 0.41) {
                if (f_posLlaUnc_h <= 0.04) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.04
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.41
                if (w_cn0Num <= 23.5) {
                  return 2;
                }
                else { // w_cn0Num > 23.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.2
              if (f_cmc_std <= 2.57) {
                if (f_max_CN0 <= 51.5) {
                  return 1;
                }
                else { // f_max_CN0 > 51.5
                  return 3;
                }
              }
              else { // f_cmc_std > 2.57
                if (pl_ekf_hor <= 0.71) {
                  return 1;
                }
                else { // pl_ekf_hor > 0.71
                  return 1;
                }
              }
            }
          }
          else { // u_ppp_use_cp_num_float > 41.5
            if (f_postCodeSTD <= 0.53) {
              if (pu_CN0greater35NumPerSys_C <= 19.5) {
                if (u_ppp_use_pr_num_float <= 35.5) {
                  return 1;
                }
                else { // u_ppp_use_pr_num_float > 35.5
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 19.5
                if (f_vdop <= 1.0) {
                  return 1;
                }
                else { // f_vdop > 1.0
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.53
              if (f_hdop <= 0.63) {
                if (w_noLLInum <= 36.5) {
                  return 1;
                }
                else { // w_noLLInum > 36.5
                  return 1;
                }
              }
              else { // f_hdop > 0.63
                if (pu_CN0greater35NumPerSys_G <= 8.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 8.5
                  return 1;
                }
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.74
          if (f_cn0_std <= 7.78) {
            if (pl_ekf_hor <= 1.42) {
              if (w_noLLInum <= 15.5) {
                if (pl_ekf_ver <= 0.11) {
                  return 1;
                }
                else { // pl_ekf_ver > 0.11
                  return 1;
                }
              }
              else { // w_noLLInum > 15.5
                if (pu_CN0greater35NumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 7.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 1.42
              if (f_postCodeSTD <= 0.92) {
                if (pu_CN0greater35NumPerSys_C <= 12.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 12.5
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.92
                if (pu_satNumPerSys_C <= 12.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 12.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_std > 7.78
            if (pl_ekf_hor <= 0.38) {
              if (f_cn0_avg <= 34.25) {
                if (pu_LLIsatNum_C <= 7.5) {
                  return 2;
                }
                else { // pu_LLIsatNum_C > 7.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 34.25
                if (f_posLlaUnc_l <= 0.03) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.03
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 0.38
              if (pl_ekf_hor <= 0.68) {
                if (f_cmc_avg <= 0.64) {
                  return 2;
                }
                else { // f_cmc_avg > 0.64
                  return 1;
                }
              }
              else { // pl_ekf_hor > 0.68
                if (f_max_CN0 <= 51.5) {
                  return 1;
                }
                else { // f_max_CN0 > 51.5
                  return 4;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.79
      if (f_posLlaUnc_l <= 0.94) {
        if (w_noLLInum <= 4.5) {
          if (f_gdop <= 2.97) {
            if (pu_CN0greater35NumPerSys_C <= 11.5) {
              if (u_MeasInUseCount <= 33.5) {
                if (pu_CN0greater35NumPerSys_G <= 2.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 2.5
                  return 3;
                }
              }
              else { // u_MeasInUseCount > 33.5
                if (u_ppp_use_pr_num_float <= 17.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 17.5
                  return 1;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 11.5
              if (f_postCodeSTD <= 0.9) {
                if (pl_ekf_ver <= 3.9) {
                  return 1;
                }
                else { // pl_ekf_ver > 3.9
                  return 3;
                }
              }
              else { // f_postCodeSTD > 0.9
                if (f_posLlaUnc_l <= 0.25) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.25
                  return 1;
                }
              }
            }
          }
          else { // f_gdop > 2.97
            if (pu_CN0greater35NumPerSys_C <= 3.5) {
              if (w_LLInum <= 30.5) {
                if (f_posLlaUnc_l <= 0.47) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.47
                  return 4;
                }
              }
              else { // w_LLInum > 30.5
                if (f_posLlaUnc_l <= 0.51) {
                  return 4;
                }
                else { // f_posLlaUnc_l > 0.51
                  return 4;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 3.5
              if (pu_CN0greater35NumPerSys_C <= 10.5) {
                if (f_posLlaUnc_b <= 0.53) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.53
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 10.5
                if (f_cmc_std <= 1.11) {
                  return 3;
                }
                else { // f_cmc_std > 1.11
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 4.5
          if (f_avg_unc_pr <= 10.47) {
            if (f_postCodeSTD <= 0.73) {
              if (f_cmc_avg <= 7.32) {
                if (pl_ekf_hor <= 1.05) {
                  return 3;
                }
                else { // pl_ekf_hor > 1.05
                  return 1;
                }
              }
              else { // f_cmc_avg > 7.32
                if (u_satMeasCount <= 21.5) {
                  return 3;
                }
                else { // u_satMeasCount > 21.5
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.73
              if (w_cn0Num <= 29.5) {
                if (pu_noLLIsatNum_G <= 5.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_G > 5.5
                  return 3;
                }
              }
              else { // w_cn0Num > 29.5
                if (pl_ekf_ver <= 1.31) {
                  return 1;
                }
                else { // pl_ekf_ver > 1.31
                  return 1;
                }
              }
            }
          }
          else { // f_avg_unc_pr > 10.47
            if (f_postPhaseSTD <= 0.01) {
              if (pu_CN0greater35NumPerSys_C <= 5.5) {
                if (pu_noLLIsatNum_G <= 6.5) {
                  return 4;
                }
                else { // pu_noLLIsatNum_G > 6.5
                  return 3;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 5.5
                if (f_posLlaUnc_l <= 0.51) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.51
                  return 4;
                }
              }
            }
            else { // f_postPhaseSTD > 0.01
              if (f_posLlaUnc_b <= 0.45) {
                if (pu_CN0greater35NumPerSys_C <= 4.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 4.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.45
                if (pl_ekf_ver <= 3.06) {
                  return 4;
                }
                else { // pl_ekf_ver > 3.06
                  return 3;
                }
              }
            }
          }
        }
      }
      else { // f_posLlaUnc_l > 0.94
        if (u_ppp_use_pr_num_float <= 9.5) {
          if (u_MeasTrackCount <= 36.5) {
            if (f_posLlaUnc_h <= 2.56) {
              if (u_satMeasCount <= 17.5) {
                if (w_cn0Num <= 20.5) {
                  return 4;
                }
                else { // w_cn0Num > 20.5
                  return 3;
                }
              }
              else { // u_satMeasCount > 17.5
                if (f_avg_unc_pr <= 9.34) {
                  return 1;
                }
                else { // f_avg_unc_pr > 9.34
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 2.56
              if (f_pdop <= 60.97) {
                if (pu_CN0greater35NumPerSys_C <= 4.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 4.5
                  return 4;
                }
              }
              else { // f_pdop > 60.97
                return 1;
              }
            }
          }
          else { // u_MeasTrackCount > 36.5
            if (pu_LLIsatNum_C <= 14.5) {
              if (f_cmc_std <= 5.49) {
                if (pu_CN0greater35NumPerSys_C <= 3.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 3.5
                  return 4;
                }
              }
              else { // f_cmc_std > 5.49
                return 3;
              }
            }
            else { // pu_LLIsatNum_C > 14.5
              if (f_cmc_std <= 0.6) {
                return 3;
              }
              else { // f_cmc_std > 0.6
                if (f_posLlaUnc_h <= 1.8) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.8
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 9.5
          if (f_cn0_avg <= 36.25) {
            if (f_posLlaUnc_b <= 0.72) {
              if (u_MeasTrackCount <= 34.0) {
                return 1;
              }
              else { // u_MeasTrackCount > 34.0
                if (f_posLlaUnc_l <= 1.7) {
                  return 3;
                }
                else { // f_posLlaUnc_l > 1.7
                  return 4;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.72
              if (f_cn0_avg <= 34.25) {
                if (f_postCodeSTD <= 1.0) {
                  return 4;
                }
                else { // f_postCodeSTD > 1.0
                  return 4;
                }
              }
              else { // f_cn0_avg > 34.25
                if (f_posLlaUnc_l <= 1.32) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 1.32
                  return 3;
                }
              }
            }
          }
          else { // f_cn0_avg > 36.25
            if (pu_noLLIsatNum_C <= 0.5) {
              if (pl_ekf_ver <= 11.29) {
                if (f_vdop <= 3.42) {
                  return 4;
                }
                else { // f_vdop > 3.42
                  return 4;
                }
              }
              else { // pl_ekf_ver > 11.29
                return 3;
              }
            }
            else { // pu_noLLIsatNum_C > 0.5
              if (f_postCodeSTD <= 0.9) {
                if (w_noLLInum <= 2.5) {
                  return 3;
                }
                else { // w_noLLInum > 2.5
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.9
                if (pu_noLLIsatNum_G <= 0.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_G > 0.5
                  return 4;
                }
              }
            }
          }
        }
      }
    }
  }
  else { // f_postCodeSTD > 1.12
    if (f_posLlaUnc_h <= 0.8) {
      if (u_CN040 <= 7.5) {
        if (w_noLLInum <= 16.5) {
          if (f_posLlaUnc_h <= 0.58) {
            if (f_cn0_avg <= 28.75) {
              if (f_max_CN0 <= 42.5) {
                if (pl_ekf_hor <= 1.83) {
                  return 1;
                }
                else { // pl_ekf_hor > 1.83
                  return 3;
                }
              }
              else { // f_max_CN0 > 42.5
                if (f_posLlaUnc_h <= 0.44) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.44
                  return 1;
                }
              }
            }
            else { // f_cn0_avg > 28.75
              if (f_max_CN0 <= 47.5) {
                if (f_hdop <= 0.63) {
                  return 3;
                }
                else { // f_hdop > 0.63
                  return 1;
                }
              }
              else { // f_max_CN0 > 47.5
                if (pu_CN0greater35NumPerSys_G <= 1.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_G > 1.5
                  return 1;
                }
              }
            }
          }
          else { // f_posLlaUnc_h > 0.58
            if (f_posLlaUnc_b <= 0.38) {
              if (u_MeasTrackCount <= 49.5) {
                if (f_posLlaUnc_b <= 0.29) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.29
                  return 1;
                }
              }
              else { // u_MeasTrackCount > 49.5
                if (pu_satNumPerSys_C <= 14.5) {
                  return 4;
                }
                else { // pu_satNumPerSys_C > 14.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.38
              if (w_LLInum <= 18.5) {
                if (pl_ekf_hor <= 3.19) {
                  return 4;
                }
                else { // pl_ekf_hor > 3.19
                  return 4;
                }
              }
              else { // w_LLInum > 18.5
                if (f_pdop <= 2.58) {
                  return 3;
                }
                else { // f_pdop > 2.58
                  return 3;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 16.5
          if (pu_noLLIsatNum_C <= 13.5) {
            if (f_posLlaUnc_h <= 0.43) {
              if (f_vdop <= 1.17) {
                if (u_ppp_use_cp_num_float <= 25.5) {
                  return 1;
                }
                else { // u_ppp_use_cp_num_float > 25.5
                  return 3;
                }
              }
              else { // f_vdop > 1.17
                if (w_cn0Num <= 38.5) {
                  return 3;
                }
                else { // w_cn0Num > 38.5
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.43
              if (f_posLlaUnc_b <= 0.16) {
                if (w_LLInum <= 17.5) {
                  return 1;
                }
                else { // w_LLInum > 17.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (f_cn0_std <= 8.53) {
                  return 1;
                }
                else { // f_cn0_std > 8.53
                  return 3;
                }
              }
            }
          }
          else { // pu_noLLIsatNum_C > 13.5
            if (f_posLlaUnc_b <= 0.11) {
              if (pu_CN0greater35NumPerSys_C <= 7.5) {
                if (u_CN040 <= 6.5) {
                  return 3;
                }
                else { // u_CN040 > 6.5
                  return 2;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 7.5
                if (u_MeasTrackCount <= 65.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 65.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.11
              if (f_max_CN0 <= 48.5) {
                if (f_cn0_avg <= 30.5) {
                  return 3;
                }
                else { // f_cn0_avg > 30.5
                  return 1;
                }
              }
              else { // f_max_CN0 > 48.5
                if (f_vdop <= 1.73) {
                  return 4;
                }
                else { // f_vdop > 1.73
                  return 3;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 7.5
        if (f_posLlaUnc_h <= 0.19) {
          if (u_MeasInUseCount <= 33.5) {
            if (f_cmc_std <= 2.15) {
              if (pl_ekf_hor <= 0.16) {
                if (pl_ekf_ver <= 0.07) {
                  return 3;
                }
                else { // pl_ekf_ver > 0.07
                  return 2;
                }
              }
              else { // pl_ekf_hor > 0.16
                if (f_posLlaUnc_b <= 0.15) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.15
                  return 4;
                }
              }
            }
            else { // f_cmc_std > 2.15
              if (f_cn0_avg <= 34.75) {
                if (pl_ekf_ver <= 0.39) {
                  return 2;
                }
                else { // pl_ekf_ver > 0.39
                  return 3;
                }
              }
              else { // f_cn0_avg > 34.75
                if (u_CN040 <= 13.5) {
                  return 3;
                }
                else { // u_CN040 > 13.5
                  return 1;
                }
              }
            }
          }
          else { // u_MeasInUseCount > 33.5
            if (f_gdop <= 1.2) {
              if (f_cn0_avg <= 41.5) {
                if (f_cn0_avg <= 36.0) {
                  return 1;
                }
                else { // f_cn0_avg > 36.0
                  return 2;
                }
              }
              else { // f_cn0_avg > 41.5
                return 1;
              }
            }
            else { // f_gdop > 1.2
              if (u_ppp_use_pr_num_float <= 27.5) {
                if (pl_ekf_hor <= 0.21) {
                  return 1;
                }
                else { // pl_ekf_hor > 0.21
                  return 1;
                }
              }
              else { // u_ppp_use_pr_num_float > 27.5
                if (u_ppp_use_pr_num_float <= 34.5) {
                  return 1;
                }
                else { // u_ppp_use_pr_num_float > 34.5
                  return 1;
                }
              }
            }
          }
        }
        else { // f_posLlaUnc_h > 0.19
          if (f_posLlaUnc_h <= 0.62) {
            if (w_LLInum <= 10.5) {
              if (f_posLlaUnc_b <= 0.14) {
                if (f_postCodeSTD <= 2.73) {
                  return 1;
                }
                else { // f_postCodeSTD > 2.73
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.14
                if (f_avg_unc_pr <= 8.81) {
                  return 1;
                }
                else { // f_avg_unc_pr > 8.81
                  return 4;
                }
              }
            }
            else { // w_LLInum > 10.5
              if (f_posLlaUnc_b <= 0.19) {
                if (f_cmc_std <= 2.3) {
                  return 1;
                }
                else { // f_cmc_std > 2.3
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.19
                if (f_max_CN0 <= 50.5) {
                  return 1;
                }
                else { // f_max_CN0 > 50.5
                  return 4;
                }
              }
            }
          }
          else { // f_posLlaUnc_h > 0.62
            if (pl_ekf_ver <= 1.26) {
              if (f_max_CN0 <= 48.5) {
                if (q_QRcheckStatus <= 0.5) {
                  return 1;
                }
                else { // q_QRcheckStatus > 0.5
                  return 3;
                }
              }
              else { // f_max_CN0 > 48.5
                if (pu_noLLIsatNum_C <= 14.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_C > 14.5
                  return 3;
                }
              }
            }
            else { // pl_ekf_ver > 1.26
              if (pl_ekf_hor <= 2.25) {
                if (u_MeasTrackCount <= 45.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 45.5
                  return 3;
                }
              }
              else { // pl_ekf_hor > 2.25
                if (u_MeasTrackCount <= 58.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 58.5
                  return 4;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.8
      if (pl_ekf_hor <= 3.15) {
        if (pu_CN0greater35NumPerSys_C <= 4.5) {
          if (pu_satNumPerSys_C <= 14.5) {
            if (pl_ekf_hor <= 2.35) {
              if (u_MeasTrackCount <= 46.5) {
                if (pu_CN0greater35NumPerSys_C <= 3.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 3.5
                  return 3;
                }
              }
              else { // u_MeasTrackCount > 46.5
                if (f_posLlaUnc_b <= 0.39) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.39
                  return 3;
                }
              }
            }
            else { // pl_ekf_hor > 2.35
              if (f_max_CN0 <= 48.5) {
                if (w_cn0Num <= 31.5) {
                  return 1;
                }
                else { // w_cn0Num > 31.5
                  return 3;
                }
              }
              else { // f_max_CN0 > 48.5
                if (pu_satNumPerSys_C <= 13.5) {
                  return 4;
                }
                else { // pu_satNumPerSys_C > 13.5
                  return 3;
                }
              }
            }
          }
          else { // pu_satNumPerSys_C > 14.5
            if (f_cn0_avg <= 27.75) {
              if (w_noLLInum <= 0.5) {
                if (pu_satNumPerSys_G <= 9.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_G > 9.5
                  return 4;
                }
              }
              else { // w_noLLInum > 0.5
                if (f_max_CN0 <= 43.5) {
                  return 3;
                }
                else { // f_max_CN0 > 43.5
                  return 1;
                }
              }
            }
            else { // f_cn0_avg > 27.75
              if (w_LLInum <= 27.5) {
                if (f_avg_unc_pr <= 7.56) {
                  return 1;
                }
                else { // f_avg_unc_pr > 7.56
                  return 1;
                }
              }
              else { // w_LLInum > 27.5
                if (f_vdop <= 3.16) {
                  return 3;
                }
                else { // f_vdop > 3.16
                  return 1;
                }
              }
            }
          }
        }
        else { // pu_CN0greater35NumPerSys_C > 4.5
          if (pl_ekf_ver <= 1.13) {
            if (u_MeasInUseCount <= 41.5) {
              if (f_max_CN0 <= 49.5) {
                if (f_cmc_avg <= 5.45) {
                  return 1;
                }
                else { // f_cmc_avg > 5.45
                  return 3;
                }
              }
              else { // f_max_CN0 > 49.5
                if (f_posLlaUnc_b <= 0.28) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.28
                  return 3;
                }
              }
            }
            else { // u_MeasInUseCount > 41.5
              if (f_cn0_std <= 5.56) {
                if (f_posLlaUnc_h <= 0.97) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.97
                  return 1;
                }
              }
              else { // f_cn0_std > 5.56
                if (f_posLlaUnc_h <= 0.87) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.87
                  return 3;
                }
              }
            }
          }
          else { // pl_ekf_ver > 1.13
            if (w_cn0Num <= 38.5) {
              if (pu_satNumPerSys_C <= 14.5) {
                if (f_posLlaUnc_b <= 0.28) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.28
                  return 3;
                }
              }
              else { // pu_satNumPerSys_C > 14.5
                if (pl_ekf_ver <= 1.53) {
                  return 3;
                }
                else { // pl_ekf_ver > 1.53
                  return 3;
                }
              }
            }
            else { // w_cn0Num > 38.5
              if (pl_ekf_hor <= 2.25) {
                if (f_cmc_std <= 0.77) {
                  return 3;
                }
                else { // f_cmc_std > 0.77
                  return 1;
                }
              }
              else { // pl_ekf_hor > 2.25
                if (f_postCodeSTD <= 1.96) {
                  return 4;
                }
                else { // f_postCodeSTD > 1.96
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 3.15
        if (u_ppp_use_pr_num_float <= 12.5) {
          if (w_LLInum <= 12.5) {
            if (f_gdop <= 12.49) {
              if (pl_ekf_hor <= 4.23) {
                if (f_cn0_std <= 8.53) {
                  return 3;
                }
                else { // f_cn0_std > 8.53
                  return 4;
                }
              }
              else { // pl_ekf_hor > 4.23
                if (f_max_CN0 <= 35.5) {
                  return 1;
                }
                else { // f_max_CN0 > 35.5
                  return 4;
                }
              }
            }
            else { // f_gdop > 12.49
              if (f_posLlaUnc_b <= 1.35) {
                return 3;
              }
              else { // f_posLlaUnc_b > 1.35
                return 4;
              }
            }
          }
          else { // w_LLInum > 12.5
            if (f_max_CN0 <= 42.5) {
              if (pu_satNumPerSys_G <= 5.5) {
                if (u_MeasTrackCount <= 30.5) {
                  return 4;
                }
                else { // u_MeasTrackCount > 30.5
                  return 1;
                }
              }
              else { // pu_satNumPerSys_G > 5.5
                if (f_cn0_avg <= 35.25) {
                  return 4;
                }
                else { // f_cn0_avg > 35.25
                  return 3;
                }
              }
            }
            else { // f_max_CN0 > 42.5
              if (f_posLlaUnc_l <= 0.39) {
                if (f_posLlaUnc_b <= 0.62) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.62
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.39
                if (f_posLlaUnc_b <= 0.36) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.36
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 12.5
          if (u_MeasTrackCount <= 35.5) {
            if (f_posLlaUnc_h <= 1.57) {
              if (f_vdop <= 1.5) {
                return 1;
              }
              else { // f_vdop > 1.5
                return 1;
              }
            }
            else { // f_posLlaUnc_h > 1.57
              if (u_MeasTrackCount <= 28.5) {
                if (pu_LLIsatNum_C <= 11.5) {
                  return 1;
                }
                else { // pu_LLIsatNum_C > 11.5
                  return 1;
                }
              }
              else { // u_MeasTrackCount > 28.5
                if (f_vdop <= 2.02) {
                  return 1;
                }
                else { // f_vdop > 2.02
                  return 3;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 35.5
            if (f_posLlaUnc_h <= 1.37) {
              if (f_posLlaUnc_h <= 1.09) {
                if (u_CN040 <= 14.5) {
                  return 1;
                }
                else { // u_CN040 > 14.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_h > 1.09
                if (u_satMeasCount <= 23.5) {
                  return 3;
                }
                else { // u_satMeasCount > 23.5
                  return 4;
                }
              }
            }
            else { // f_posLlaUnc_h > 1.37
              if (f_cn0_avg <= 38.75) {
                if (f_posLlaUnc_b <= 0.69) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.69
                  return 3;
                }
              }
              else { // f_cn0_avg > 38.75
                if (u_ppp_use_pr_num_float <= 15.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 15.5
                  return 1;
                }
              }
            }
          }
        }
      }
    }
  }
}

static uint8_t predict_scene_2(integ_PppFeature_t* gpz_pppFeature) {
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  uint8_t u_ppp_use_cp_num_float = gpz_pppFeature->u_ppp_use_cp_num_float;
  uint16_t w_cn0Num = gpz_pppFeature->w_cn0Num;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_posLlaUnc_b = gpz_pppFeature->f_posLlaUnc[0];
  uint8_t pu_CN0greater35NumPerSys_C = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_BDS3];
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t pu_noLLIsatNum_C = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_BDS3];
  float f_avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t pu_noLLIsatNum_G = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_GPS];
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  int32_t q_QRcheckStatus = (int32_t)gpz_pppFeature->q_QRcheckStatus;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  float f_posLlaUnc_h = gpz_pppFeature->f_posLlaUnc[2];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_posLlaUnc_l = gpz_pppFeature->f_posLlaUnc[1];
  uint8_t pu_satNumPerSys_G = gpz_pppFeature->pu_satNumPerSys[C_GNSS_GPS];
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  uint16_t w_LLInum = gpz_pppFeature->w_LLInum;
  uint8_t pu_LLIsatNum_C = gpz_pppFeature->pu_LLIsatNum[C_GNSS_BDS3];
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  uint8_t pu_CN0greater35NumPerSys_G = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_GPS];
  uint8_t pu_satNumPerSys_C = gpz_pppFeature->pu_satNumPerSys[C_GNSS_BDS3];
  float f_pdop = gpz_pppFeature->z_dops.f_pdop;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;
  float f_max_CN0 = gpz_pppFeature->f_max_CN0;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  uint16_t w_noLLInum = gpz_pppFeature->w_noLLInum;

  if (f_postCodeSTD <= 1.12) {
    if (f_posLlaUnc_h <= 0.79) {
      if (u_CN040 <= 8.5) {
        if (w_noLLInum <= 18.5) {
          if (f_cmc_avg <= 3.29) {
            if (f_posLlaUnc_h <= 0.63) {
              if (pu_noLLIsatNum_G <= 4.5) {
                if (f_posLlaUnc_l <= 0.39) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.39
                  return 1;
                }
              }
              else { // pu_noLLIsatNum_G > 4.5
                if (pu_satNumPerSys_C <= 17.0) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 17.0
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.63
              if (w_noLLInum <= 2.5) {
                if (pu_satNumPerSys_C <= 20.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 20.5
                  return 3;
                }
              }
              else { // w_noLLInum > 2.5
                if (f_max_CN0 <= 49.5) {
                  return 1;
                }
                else { // f_max_CN0 > 49.5
                  return 3;
                }
              }
            }
          }
          else { // f_cmc_avg > 3.29
            if (f_gdop <= 3.95) {
              if (f_cmc_avg <= 6.64) {
                if (u_ppp_use_pr_num_float <= 13.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 13.5
                  return 1;
                }
              }
              else { // f_cmc_avg > 6.64
                return 2;
              }
            }
            else { // f_gdop > 3.95
              return 1;
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (pu_CN0greater35NumPerSys_C <= 6.5) {
            if (f_posLlaUnc_b <= 0.11) {
              if (f_vdop <= 1.12) {
                return 1;
              }
              else { // f_vdop > 1.12
                if (u_MeasInUseCount <= 31.5) {
                  return 3;
                }
                else { // u_MeasInUseCount > 31.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.11
              if (pu_LLIsatNum_C <= 3.5) {
                if (pu_noLLIsatNum_C <= 11.5) {
                  return 3;
                }
                else { // pu_noLLIsatNum_C > 11.5
                  return 4;
                }
              }
              else { // pu_LLIsatNum_C > 3.5
                if (pu_satNumPerSys_G <= 7.5) {
                  return 2;
                }
                else { // pu_satNumPerSys_G > 7.5
                  return 1;
                }
              }
            }
          }
          else { // pu_CN0greater35NumPerSys_C > 6.5
            if (f_max_CN0 <= 49.5) {
              if (f_postPhaseSTD <= 0.03) {
                if (f_avg_unc_pr <= 3.93) {
                  return 3;
                }
                else { // f_avg_unc_pr > 3.93
                  return 1;
                }
              }
              else { // f_postPhaseSTD > 0.03
                return 2;
              }
            }
            else { // f_max_CN0 > 49.5
              if (f_posLlaUnc_b <= 0.11) {
                if (u_MeasTrackCount <= 52.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 52.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.11
                if (f_posLlaUnc_b <= 0.14) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.14
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 8.5
        if (f_postCodeSTD <= 0.74) {
          if (u_ppp_use_cp_num_float <= 41.5) {
            if (pl_ekf_ver <= 0.2) {
              if (f_postCodeSTD <= 0.41) {
                if (f_posLlaUnc_h <= 0.04) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.04
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.41
                if (w_cn0Num <= 23.5) {
                  return 2;
                }
                else { // w_cn0Num > 23.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.2
              if (f_cmc_std <= 2.62) {
                if (f_max_CN0 <= 51.5) {
                  return 1;
                }
                else { // f_max_CN0 > 51.5
                  return 3;
                }
              }
              else { // f_cmc_std > 2.62
                if (pl_ekf_hor <= 0.71) {
                  return 1;
                }
                else { // pl_ekf_hor > 0.71
                  return 1;
                }
              }
            }
          }
          else { // u_ppp_use_cp_num_float > 41.5
            if (f_postCodeSTD <= 0.53) {
              if (pu_CN0greater35NumPerSys_C <= 19.5) {
                if (u_ppp_use_pr_num_float <= 35.5) {
                  return 1;
                }
                else { // u_ppp_use_pr_num_float > 35.5
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 19.5
                if (f_vdop <= 1.0) {
                  return 1;
                }
                else { // f_vdop > 1.0
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.53
              if (f_hdop <= 0.63) {
                if (f_posLlaUnc_b <= 0.01) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.01
                  return 1;
                }
              }
              else { // f_hdop > 0.63
                if (pu_CN0greater35NumPerSys_G <= 8.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 8.5
                  return 1;
                }
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.74
          if (f_cn0_std <= 7.78) {
            if (pl_ekf_hor <= 1.42) {
              if (w_noLLInum <= 15.5) {
                if (pl_ekf_ver <= 0.11) {
                  return 1;
                }
                else { // pl_ekf_ver > 0.11
                  return 1;
                }
              }
              else { // w_noLLInum > 15.5
                if (pu_CN0greater35NumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 7.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 1.42
              if (f_postCodeSTD <= 0.92) {
                if (f_pdop <= 3.27) {
                  return 1;
                }
                else { // f_pdop > 3.27
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.92
                if (pu_satNumPerSys_C <= 12.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 12.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_std > 7.78
            if (pl_ekf_hor <= 0.38) {
              if (f_cn0_avg <= 34.25) {
                if (pu_LLIsatNum_C <= 7.5) {
                  return 2;
                }
                else { // pu_LLIsatNum_C > 7.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 34.25
                if (f_posLlaUnc_l <= 0.03) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.03
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 0.38
              if (pl_ekf_hor <= 0.68) {
                if (f_cmc_avg <= 0.64) {
                  return 2;
                }
                else { // f_cmc_avg > 0.64
                  return 1;
                }
              }
              else { // pl_ekf_hor > 0.68
                if (f_max_CN0 <= 51.5) {
                  return 1;
                }
                else { // f_max_CN0 > 51.5
                  return 4;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.79
      if (pl_ekf_ver <= 4.16) {
        if (w_noLLInum <= 4.5) {
          if (f_gdop <= 2.97) {
            if (pu_CN0greater35NumPerSys_C <= 12.5) {
              if (f_posLlaUnc_l <= 0.28) {
                if (u_MeasTrackCount <= 51.5) {
                  return 3;
                }
                else { // u_MeasTrackCount > 51.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.28
                if (pl_ekf_hor <= 2.95) {
                  return 3;
                }
                else { // pl_ekf_hor > 2.95
                  return 1;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 12.5
              if (f_postCodeSTD <= 0.92) {
                if (u_ppp_use_pr_num_float <= 19.5) {
                  return 1;
                }
                else { // u_ppp_use_pr_num_float > 19.5
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.92
                if (pl_ekf_ver <= 1.29) {
                  return 1;
                }
                else { // pl_ekf_ver > 1.29
                  return 1;
                }
              }
            }
          }
          else { // f_gdop > 2.97
            if (pu_CN0greater35NumPerSys_C <= 4.5) {
              if (w_cn0Num <= 31.5) {
                if (w_noLLInum <= 0.5) {
                  return 4;
                }
                else { // w_noLLInum > 0.5
                  return 1;
                }
              }
              else { // w_cn0Num > 31.5
                if (f_postPhaseSTD <= 0.0) {
                  return 4;
                }
                else { // f_postPhaseSTD > 0.0
                  return 3;
                }
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 4.5
              if (pu_CN0greater35NumPerSys_C <= 10.5) {
                if (f_posLlaUnc_b <= 0.51) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.51
                  return 1;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 10.5
                if (f_cmc_std <= 1.14) {
                  return 3;
                }
                else { // f_cmc_std > 1.14
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 4.5
          if (f_avg_unc_pr <= 10.47) {
            if (f_postCodeSTD <= 0.74) {
              if (f_cmc_avg <= 7.32) {
                if (pl_ekf_hor <= 1.07) {
                  return 3;
                }
                else { // pl_ekf_hor > 1.07
                  return 1;
                }
              }
              else { // f_cmc_avg > 7.32
                if (pu_satNumPerSys_C <= 18.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 18.5
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.74
              if (w_cn0Num <= 29.5) {
                if (pu_noLLIsatNum_G <= 5.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_G > 5.5
                  return 3;
                }
              }
              else { // w_cn0Num > 29.5
                if (f_posLlaUnc_l <= 0.3) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.3
                  return 1;
                }
              }
            }
          }
          else { // f_avg_unc_pr > 10.47
            if (f_postPhaseSTD <= 0.01) {
              if (pu_satNumPerSys_C <= 18.5) {
                if (pu_noLLIsatNum_G <= 6.5) {
                  return 4;
                }
                else { // pu_noLLIsatNum_G > 6.5
                  return 3;
                }
              }
              else { // pu_satNumPerSys_C > 18.5
                if (pl_ekf_ver <= 1.15) {
                  return 3;
                }
                else { // pl_ekf_ver > 1.15
                  return 1;
                }
              }
            }
            else { // f_postPhaseSTD > 0.01
              if (f_posLlaUnc_b <= 0.45) {
                if (pu_satNumPerSys_C <= 15.0) {
                  return 4;
                }
                else { // pu_satNumPerSys_C > 15.0
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.45
                if (pu_CN0greater35NumPerSys_C <= 6.0) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 6.0
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 4.16
        if (u_ppp_use_pr_num_float <= 9.5) {
          if (u_MeasTrackCount <= 36.5) {
            if (f_posLlaUnc_h <= 2.48) {
              if (u_satMeasCount <= 17.5) {
                if (f_cmc_std <= 0.87) {
                  return 3;
                }
                else { // f_cmc_std > 0.87
                  return 1;
                }
              }
              else { // u_satMeasCount > 17.5
                if (f_pdop <= 2.42) {
                  return 1;
                }
                else { // f_pdop > 2.42
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 2.48
              if (f_gdop <= 74.92) {
                if (f_posLlaUnc_h <= 15.48) {
                  return 4;
                }
                else { // f_posLlaUnc_h > 15.48
                  return 3;
                }
              }
              else { // f_gdop > 74.92
                if (f_cmc_avg <= 0.32) {
                  return 1;
                }
                else { // f_cmc_avg > 0.32
                  return 1;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 36.5
            if (pu_LLIsatNum_C <= 14.5) {
              if (f_cmc_std <= 5.49) {
                if (pu_LLIsatNum_C <= 11.5) {
                  return 4;
                }
                else { // pu_LLIsatNum_C > 11.5
                  return 4;
                }
              }
              else { // f_cmc_std > 5.49
                return 3;
              }
            }
            else { // pu_LLIsatNum_C > 14.5
              if (f_cmc_std <= 0.6) {
                return 3;
              }
              else { // f_cmc_std > 0.6
                if (f_posLlaUnc_h <= 1.79) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.79
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 9.5
          if (f_cn0_avg <= 36.25) {
            if (f_posLlaUnc_b <= 0.72) {
              if (pl_ekf_hor <= 7.78) {
                if (w_LLInum <= 16.5) {
                  return 1;
                }
                else { // w_LLInum > 16.5
                  return 3;
                }
              }
              else { // pl_ekf_hor > 7.78
                return 4;
              }
            }
            else { // f_posLlaUnc_b > 0.72
              if (f_cn0_avg <= 34.25) {
                if (f_postCodeSTD <= 1.0) {
                  return 4;
                }
                else { // f_postCodeSTD > 1.0
                  return 4;
                }
              }
              else { // f_cn0_avg > 34.25
                if (f_posLlaUnc_h <= 2.38) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 2.38
                  return 3;
                }
              }
            }
          }
          else { // f_cn0_avg > 36.25
            if (w_cn0Num <= 19.5) {
              if (f_vdop <= 2.72) {
                if (f_postCodeSTD <= 0.85) {
                  return 1;
                }
                else { // f_postCodeSTD > 0.85
                  return 3;
                }
              }
              else { // f_vdop > 2.72
                if (f_posLlaUnc_l <= 2.56) {
                  return 4;
                }
                else { // f_posLlaUnc_l > 2.56
                  return 3;
                }
              }
            }
            else { // w_cn0Num > 19.5
              if (f_hdop <= 0.82) {
                if (f_avg_unc_pr <= 5.32) {
                  return 3;
                }
                else { // f_avg_unc_pr > 5.32
                  return 1;
                }
              }
              else { // f_hdop > 0.82
                if (w_noLLInum <= 3.5) {
                  return 1;
                }
                else { // w_noLLInum > 3.5
                  return 1;
                }
              }
            }
          }
        }
      }
    }
  }
  else { // f_postCodeSTD > 1.12
    if (f_posLlaUnc_h <= 0.67) {
      if (u_CN040 <= 7.5) {
        if (w_noLLInum <= 16.5) {
          if (f_cn0_avg <= 28.75) {
            if (pu_noLLIsatNum_G <= 2.5) {
              if (f_avg_unc_pr <= 18.54) {
                if (f_posLlaUnc_b <= 0.37) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.37
                  return 3;
                }
              }
              else { // f_avg_unc_pr > 18.54
                if (pu_satNumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_G > 7.5
                  return 3;
                }
              }
            }
            else { // pu_noLLIsatNum_G > 2.5
              if (pu_LLIsatNum_C <= 12.5) {
                if (pl_ekf_ver <= 0.48) {
                  return 3;
                }
                else { // pl_ekf_ver > 0.48
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 12.5
                if (f_cn0_std <= 8.53) {
                  return 3;
                }
                else { // f_cn0_std > 8.53
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_avg > 28.75
            if (pl_ekf_ver <= 0.57) {
              if (f_posLlaUnc_h <= 0.47) {
                if (f_cmc_avg <= 2.77) {
                  return 1;
                }
                else { // f_cmc_avg > 2.77
                  return 1;
                }
              }
              else { // f_posLlaUnc_h > 0.47
                if (pl_ekf_hor <= 0.84) {
                  return 2;
                }
                else { // pl_ekf_hor > 0.84
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.57
              if (w_noLLInum <= 7.5) {
                if (f_posLlaUnc_h <= 0.57) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.57
                  return 1;
                }
              }
              else { // w_noLLInum > 7.5
                if (u_satMeasCount <= 24.5) {
                  return 1;
                }
                else { // u_satMeasCount > 24.5
                  return 3;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 16.5
          if (pu_noLLIsatNum_C <= 13.5) {
            if (f_posLlaUnc_h <= 0.43) {
              if (f_vdop <= 1.17) {
                if (u_ppp_use_cp_num_float <= 25.5) {
                  return 1;
                }
                else { // u_ppp_use_cp_num_float > 25.5
                  return 3;
                }
              }
              else { // f_vdop > 1.17
                if (u_MeasTrackCount <= 59.0) {
                  return 3;
                }
                else { // u_MeasTrackCount > 59.0
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.43
              if (f_posLlaUnc_b <= 0.16) {
                if (w_LLInum <= 17.5) {
                  return 1;
                }
                else { // w_LLInum > 17.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 6.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 6.5
                  return 1;
                }
              }
            }
          }
          else { // pu_noLLIsatNum_C > 13.5
            if (f_posLlaUnc_b <= 0.11) {
              if (pu_CN0greater35NumPerSys_C <= 7.5) {
                if (u_CN040 <= 6.5) {
                  return 3;
                }
                else { // u_CN040 > 6.5
                  return 2;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 7.5
                if (u_MeasTrackCount <= 65.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 65.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.11
              if (f_max_CN0 <= 48.5) {
                if (f_cn0_avg <= 30.5) {
                  return 3;
                }
                else { // f_cn0_avg > 30.5
                  return 1;
                }
              }
              else { // f_max_CN0 > 48.5
                if (f_vdop <= 1.73) {
                  return 4;
                }
                else { // f_vdop > 1.73
                  return 3;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 7.5
        if (w_noLLInum <= 18.5) {
          if (pl_ekf_hor <= 1.24) {
            if (f_cmc_std <= 2.09) {
              if (f_cn0_avg <= 43.25) {
                if (w_noLLInum <= 11.5) {
                  return 1;
                }
                else { // w_noLLInum > 11.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 43.25
                if (f_posLlaUnc_h <= 0.4) {
                  return 4;
                }
                else { // f_posLlaUnc_h > 0.4
                  return 3;
                }
              }
            }
            else { // f_cmc_std > 2.09
              if (pl_ekf_hor <= 0.3) {
                if (f_cmc_avg <= 5.02) {
                  return 1;
                }
                else { // f_cmc_avg > 5.02
                  return 2;
                }
              }
              else { // pl_ekf_hor > 0.3
                if (u_MeasTrackCount <= 65.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 65.5
                  return 3;
                }
              }
            }
          }
          else { // pl_ekf_hor > 1.24
            if (q_QRcheckStatus <= 0.5) {
              if (f_cn0_avg <= 29.25) {
                if (pl_ekf_ver <= 1.02) {
                  return 3;
                }
                else { // pl_ekf_ver > 1.02
                  return 1;
                }
              }
              else { // f_cn0_avg > 29.25
                if (w_noLLInum <= 11.5) {
                  return 1;
                }
                else { // w_noLLInum > 11.5
                  return 1;
                }
              }
            }
            else { // q_QRcheckStatus > 0.5
              if (pl_ekf_hor <= 1.51) {
                if (f_posLlaUnc_l <= 0.15) {
                  return 3;
                }
                else { // f_posLlaUnc_l > 0.15
                  return 1;
                }
              }
              else { // pl_ekf_hor > 1.51
                if (f_posLlaUnc_h <= 0.56) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.56
                  return 3;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (f_cn0_avg <= 38.25) {
            if (f_posLlaUnc_h <= 0.18) {
              if (f_cn0_avg <= 34.75) {
                if (pl_ekf_ver <= 0.39) {
                  return 2;
                }
                else { // pl_ekf_ver > 0.39
                  return 3;
                }
              }
              else { // f_cn0_avg > 34.75
                if (u_CN040 <= 13.5) {
                  return 3;
                }
                else { // u_CN040 > 13.5
                  return 2;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.18
              if (f_posLlaUnc_b <= 0.16) {
                if (f_postCodeSTD <= 2.8) {
                  return 1;
                }
                else { // f_postCodeSTD > 2.8
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 8.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 8.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_avg > 38.25
            if (f_postCodeSTD <= 1.75) {
              if (f_posLlaUnc_b <= 0.15) {
                if (f_gdop <= 1.2) {
                  return 2;
                }
                else { // f_gdop > 1.2
                  return 1;
                }
              }
              else { // f_posLlaUnc_b > 0.15
                if (u_ppp_use_pr_num_float <= 16.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 16.5
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 1.75
              if (f_posLlaUnc_l <= 0.02) {
                if (w_cn0Num <= 37.5) {
                  return 2;
                }
                else { // w_cn0Num > 37.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.02
                if (f_vdop <= 1.13) {
                  return 2;
                }
                else { // f_vdop > 1.13
                  return 1;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.67
      if (pl_ekf_hor <= 3.15) {
        if (pl_ekf_ver <= 1.3) {
          if (f_cn0_avg <= 29.75) {
            if (pu_satNumPerSys_C <= 14.5) {
              if (u_satMeasCount <= 17.5) {
                return 1;
              }
              else { // u_satMeasCount > 17.5
                return 4;
              }
            }
            else { // pu_satNumPerSys_C > 14.5
              if (pu_LLIsatNum_C <= 17.5) {
                if (f_posLlaUnc_b <= 0.35) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.35
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 17.5
                if (f_posLlaUnc_b <= 0.42) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.42
                  return 3;
                }
              }
            }
          }
          else { // f_cn0_avg > 29.75
            if (f_cn0_std <= 5.56) {
              if (f_posLlaUnc_l <= 0.24) {
                if (u_MeasInUseCount <= 39.5) {
                  return 1;
                }
                else { // u_MeasInUseCount > 39.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_l > 0.24
                if (u_MeasTrackCount <= 63.5) {
                  return 3;
                }
                else { // u_MeasTrackCount > 63.5
                  return 1;
                }
              }
            }
            else { // f_cn0_std > 5.56
              if (f_max_CN0 <= 48.5) {
                if (u_MeasInUseCount <= 44.5) {
                  return 1;
                }
                else { // u_MeasInUseCount > 44.5
                  return 3;
                }
              }
              else { // f_max_CN0 > 48.5
                if (f_posLlaUnc_h <= 0.77) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.77
                  return 1;
                }
              }
            }
          }
        }
        else { // pl_ekf_ver > 1.3
          if (f_avg_unc_pr <= 8.65) {
            if (f_postCodeSTD <= 2.11) {
              if (f_posLlaUnc_b <= 0.47) {
                if (w_LLInum <= 9.5) {
                  return 3;
                }
                else { // w_LLInum > 9.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.47
                if (u_MeasTrackCount <= 52.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 52.5
                  return 4;
                }
              }
            }
            else { // f_postCodeSTD > 2.11
              if (u_MeasInUseCount <= 39.5) {
                if (f_cmc_std <= 6.28) {
                  return 3;
                }
                else { // f_cmc_std > 6.28
                  return 4;
                }
              }
              else { // u_MeasInUseCount > 39.5
                if (pl_ekf_hor <= 2.01) {
                  return 3;
                }
                else { // pl_ekf_hor > 2.01
                  return 4;
                }
              }
            }
          }
          else { // f_avg_unc_pr > 8.65
            if (pu_satNumPerSys_G <= 8.5) {
              if (w_noLLInum <= 8.5) {
                if (u_satMeasCount <= 25.5) {
                  return 3;
                }
                else { // u_satMeasCount > 25.5
                  return 4;
                }
              }
              else { // w_noLLInum > 8.5
                if (u_MeasTrackCount <= 55.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 55.5
                  return 4;
                }
              }
            }
            else { // pu_satNumPerSys_G > 8.5
              if (f_posLlaUnc_h <= 1.05) {
                if (w_noLLInum <= 14.5) {
                  return 3;
                }
                else { // w_noLLInum > 14.5
                  return 4;
                }
              }
              else { // f_posLlaUnc_h > 1.05
                if (f_postCodeSTD <= 1.51) {
                  return 3;
                }
                else { // f_postCodeSTD > 1.51
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 3.15
        if (u_ppp_use_pr_num_float <= 12.5) {
          if (w_LLInum <= 12.5) {
            if (f_gdop <= 12.49) {
              if (pl_ekf_hor <= 4.24) {
                if (f_cn0_std <= 8.53) {
                  return 3;
                }
                else { // f_cn0_std > 8.53
                  return 4;
                }
              }
              else { // pl_ekf_hor > 4.24
                if (f_max_CN0 <= 35.5) {
                  return 1;
                }
                else { // f_max_CN0 > 35.5
                  return 4;
                }
              }
            }
            else { // f_gdop > 12.49
              if (pu_noLLIsatNum_G <= 1.5) {
                return 4;
              }
              else { // pu_noLLIsatNum_G > 1.5
                return 3;
              }
            }
          }
          else { // w_LLInum > 12.5
            if (f_max_CN0 <= 40.5) {
              if (pu_satNumPerSys_G <= 5.5) {
                if (f_posLlaUnc_h <= 1.8) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.8
                  return 4;
                }
              }
              else { // pu_satNumPerSys_G > 5.5
                if (pl_ekf_ver <= 3.19) {
                  return 4;
                }
                else { // pl_ekf_ver > 3.19
                  return 4;
                }
              }
            }
            else { // f_max_CN0 > 40.5
              if (pl_ekf_ver <= 1.72) {
                if (f_posLlaUnc_h <= 1.01) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 1.01
                  return 1;
                }
              }
              else { // pl_ekf_ver > 1.72
                if (f_posLlaUnc_b <= 0.36) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.36
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 12.5
          if (u_MeasTrackCount <= 35.5) {
            if (f_posLlaUnc_h <= 1.57) {
              if (f_posLlaUnc_l <= 0.64) {
                if (f_cmc_std <= 0.53) {
                  return 1;
                }
                else { // f_cmc_std > 0.53
                  return 3;
                }
              }
              else { // f_posLlaUnc_l > 0.64
                return 1;
              }
            }
            else { // f_posLlaUnc_h > 1.57
              if (u_MeasTrackCount <= 28.5) {
                if (u_MeasInUseCount <= 19.0) {
                  return 1;
                }
                else { // u_MeasInUseCount > 19.0
                  return 1;
                }
              }
              else { // u_MeasTrackCount > 28.5
                if (pu_satNumPerSys_C <= 13.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 13.5
                  return 1;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 35.5
            if (f_posLlaUnc_h <= 1.37) {
              if (f_posLlaUnc_h <= 1.09) {
                if (u_CN040 <= 14.5) {
                  return 1;
                }
                else { // u_CN040 > 14.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_h > 1.09
                if (pu_CN0greater35NumPerSys_C <= 12.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 12.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 1.37
              if (f_cn0_avg <= 38.75) {
                if (f_posLlaUnc_b <= 0.72) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.72
                  return 3;
                }
              }
              else { // f_cn0_avg > 38.75
                if (u_ppp_use_pr_num_float <= 15.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 15.5
                  return 1;
                }
              }
            }
          }
        }
      }
    }
  }
}

static uint8_t predict_scene_3(integ_PppFeature_t* gpz_pppFeature) {
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  uint8_t u_ppp_use_cp_num_float = gpz_pppFeature->u_ppp_use_cp_num_float;
  uint16_t w_cn0Num = gpz_pppFeature->w_cn0Num;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  uint8_t u_ns = gpz_pppFeature->u_ns;
  float f_posLlaUnc_b = gpz_pppFeature->f_posLlaUnc[0];
  uint8_t pu_CN0greater35NumPerSys_C = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_BDS3];
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t pu_noLLIsatNum_C = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_BDS3];
  float f_avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t pu_noLLIsatNum_G = gpz_pppFeature->pu_noLLIsatNum[C_GNSS_GPS];
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  int32_t q_QRcheckStatus = (int32_t)gpz_pppFeature->q_QRcheckStatus;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  float f_posLlaUnc_h = gpz_pppFeature->f_posLlaUnc[2];
  float f_posLlaUnc_l = gpz_pppFeature->f_posLlaUnc[1];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  uint8_t pu_satNumPerSys_G = gpz_pppFeature->pu_satNumPerSys[C_GNSS_GPS];
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  uint16_t w_LLInum = gpz_pppFeature->w_LLInum;
  uint8_t pu_LLIsatNum_G = gpz_pppFeature->pu_LLIsatNum[C_GNSS_GPS];
  uint8_t pu_LLIsatNum_C = gpz_pppFeature->pu_LLIsatNum[C_GNSS_BDS3];
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  uint8_t pu_CN0greater35NumPerSys_G = gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_GPS];
  uint8_t pu_satNumPerSys_C = gpz_pppFeature->pu_satNumPerSys[C_GNSS_BDS3];
  float f_pdop = gpz_pppFeature->z_dops.f_pdop;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;
  float f_max_CN0 = gpz_pppFeature->f_max_CN0;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  uint16_t w_noLLInum = gpz_pppFeature->w_noLLInum;

  if (f_postCodeSTD <= 1.12) {
    if (f_posLlaUnc_h <= 0.79) {
      if (u_CN040 <= 8.5) {
        if (w_noLLInum <= 18.5) {
          if (f_posLlaUnc_h <= 0.63) {
            if (f_cmc_avg <= 3.29) {
              if (f_max_CN0 <= 48.5) {
                if (f_avg_unc_pr <= 18.85) {
                  return 1;
                }
                else { // f_avg_unc_pr > 18.85
                  return 1;
                }
              }
              else { // f_max_CN0 > 48.5
                if (pl_ekf_ver <= 0.74) {
                  return 3;
                }
                else { // pl_ekf_ver > 0.74
                  return 1;
                }
              }
            }
            else { // f_cmc_avg > 3.29
              if (f_max_CN0 <= 43.5) {
                if (pu_LLIsatNum_C <= 11.5) {
                  return 2;
                }
                else { // pu_LLIsatNum_C > 11.5
                  return 1;
                }
              }
              else { // f_max_CN0 > 43.5
                if (f_postCodeSTD <= 0.64) {
                  return 1;
                }
                else { // f_postCodeSTD > 0.64
                  return 3;
                }
              }
            }
          }
          else { // f_posLlaUnc_h > 0.63
            if (f_posLlaUnc_b <= 0.37) {
              if (pu_LLIsatNum_C <= 19.5) {
                if (f_max_CN0 <= 49.5) {
                  return 1;
                }
                else { // f_max_CN0 > 49.5
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 19.5
                if (u_MeasTrackCount <= 56.5) {
                  return 3;
                }
                else { // u_MeasTrackCount > 56.5
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.37
              if (pu_noLLIsatNum_C <= 4.5) {
                if (f_posLlaUnc_h <= 0.74) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.74
                  return 3;
                }
              }
              else { // pu_noLLIsatNum_C > 4.5
                return 1;
              }
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (f_posLlaUnc_h <= 0.29) {
            if (pu_CN0greater35NumPerSys_C <= 6.5) {
              if (f_posLlaUnc_b <= 0.12) {
                if (f_vdop <= 1.12) {
                  return 1;
                }
                else { // f_vdop > 1.12
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.12
                return 4;
              }
            }
            else { // pu_CN0greater35NumPerSys_C > 6.5
              if (f_posLlaUnc_b <= 0.11) {
                if (f_cmc_avg <= 2.71) {
                  return 1;
                }
                else { // f_cmc_avg > 2.71
                  return 2;
                }
              }
              else { // f_posLlaUnc_b > 0.11
                if (f_vdop <= 1.5) {
                  return 4;
                }
                else { // f_vdop > 1.5
                  return 3;
                }
              }
            }
          }
          else { // f_posLlaUnc_h > 0.29
            if (f_max_CN0 <= 49.5) {
              if (f_cn0_avg <= 29.75) {
                if (u_CN040 <= 6.5) {
                  return 3;
                }
                else { // u_CN040 > 6.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 29.75
                if (u_MeasTrackCount <= 50.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 50.5
                  return 1;
                }
              }
            }
            else { // f_max_CN0 > 49.5
              if (f_cmc_avg <= 2.48) {
                if (f_postCodeSTD <= 0.66) {
                  return 3;
                }
                else { // f_postCodeSTD > 0.66
                  return 4;
                }
              }
              else { // f_cmc_avg > 2.48
                if (pu_LLIsatNum_C <= 4.0) {
                  return 3;
                }
                else { // pu_LLIsatNum_C > 4.0
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 8.5
        if (f_postCodeSTD <= 0.77) {
          if (f_postCodeSTD <= 0.41) {
            if (u_ppp_use_cp_num_float <= 41.5) {
              if (u_ns <= 24.5) {
                if (w_cn0Num <= 20.5) {
                  return 2;
                }
                else { // w_cn0Num > 20.5
                  return 1;
                }
              }
              else { // u_ns > 24.5
                if (f_hdop <= 0.62) {
                  return 1;
                }
                else { // f_hdop > 0.62
                  return 1;
                }
              }
            }
            else { // u_ppp_use_cp_num_float > 41.5
              if (u_ppp_use_pr_num_float <= 35.5) {
                if (f_hdop <= 0.61) {
                  return 1;
                }
                else { // f_hdop > 0.61
                  return 1;
                }
              }
              else { // u_ppp_use_pr_num_float > 35.5
                if (pu_CN0greater35NumPerSys_C <= 12.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 12.5
                  return 1;
                }
              }
            }
          }
          else { // f_postCodeSTD > 0.41
            if (pl_ekf_ver <= 0.2) {
              if (f_posLlaUnc_h <= 0.04) {
                if (u_ns <= 23.5) {
                  return 1;
                }
                else { // u_ns > 23.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_h > 0.04
                if (w_cn0Num <= 23.5) {
                  return 2;
                }
                else { // w_cn0Num > 23.5
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.2
              if (u_MeasInUseCount <= 36.5) {
                if (pl_ekf_hor <= 0.71) {
                  return 1;
                }
                else { // pl_ekf_hor > 0.71
                  return 1;
                }
              }
              else { // u_MeasInUseCount > 36.5
                if (f_posLlaUnc_h <= 0.13) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.13
                  return 1;
                }
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.77
          if (f_cn0_std <= 7.78) {
            if (pl_ekf_hor <= 1.42) {
              if (w_LLInum <= 19.5) {
                if (pu_CN0greater35NumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_G > 7.5
                  return 1;
                }
              }
              else { // w_LLInum > 19.5
                if (pl_ekf_hor <= 0.18) {
                  return 1;
                }
                else { // pl_ekf_hor > 0.18
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 1.42
              if (f_postCodeSTD <= 0.91) {
                if (pu_CN0greater35NumPerSys_C <= 13.5) {
                  return 1;
                }
                else { // pu_CN0greater35NumPerSys_C > 13.5
                  return 1;
                }
              }
              else { // f_postCodeSTD > 0.91
                if (pu_satNumPerSys_C <= 12.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 12.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_std > 7.78
            if (pl_ekf_hor <= 0.54) {
              if (u_CN040 <= 12.5) {
                if (f_posLlaUnc_b <= 0.03) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.03
                  return 2;
                }
              }
              else { // u_CN040 > 12.5
                if (f_posLlaUnc_l <= 0.03) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.03
                  return 1;
                }
              }
            }
            else { // pl_ekf_hor > 0.54
              if (w_LLInum <= 7.5) {
                if (f_cmc_avg <= 0.66) {
                  return 2;
                }
                else { // f_cmc_avg > 0.66
                  return 1;
                }
              }
              else { // w_LLInum > 7.5
                if (f_posLlaUnc_h <= 0.65) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.65
                  return 1;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.79
      if (f_posLlaUnc_l <= 0.94) {
        if (f_postCodeSTD <= 0.7) {
          if (f_cn0_avg <= 30.75) {
            if (pu_LLIsatNum_G <= 3.5) {
              return 3;
            }
            else { // pu_LLIsatNum_G > 3.5
              if (u_MeasInUseCount <= 25.5) {
                if (f_posLlaUnc_l <= 0.52) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.52
                  return 4;
                }
              }
              else { // u_MeasInUseCount > 25.5
                if (f_posLlaUnc_b <= 0.43) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.43
                  return 3;
                }
              }
            }
          }
          else { // f_cn0_avg > 30.75
            if (pu_noLLIsatNum_C <= 2.5) {
              if (u_ppp_use_pr_num_float <= 17.5) {
                if (f_max_CN0 <= 46.5) {
                  return 3;
                }
                else { // f_max_CN0 > 46.5
                  return 1;
                }
              }
              else { // u_ppp_use_pr_num_float > 17.5
                if (u_ppp_use_cp_num_float <= 19.5) {
                  return 1;
                }
                else { // u_ppp_use_cp_num_float > 19.5
                  return 1;
                }
              }
            }
            else { // pu_noLLIsatNum_C > 2.5
              if (pl_ekf_hor <= 1.04) {
                if (u_MeasInUseCount <= 27.5) {
                  return 3;
                }
                else { // u_MeasInUseCount > 27.5
                  return 1;
                }
              }
              else { // pl_ekf_hor > 1.04
                if (pl_ekf_hor <= 3.07) {
                  return 1;
                }
                else { // pl_ekf_hor > 3.07
                  return 1;
                }
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.7
          if (pu_CN0greater35NumPerSys_C <= 12.5) {
            if (w_noLLInum <= 4.5) {
              if (pl_ekf_hor <= 3.18) {
                if (pl_ekf_ver <= 1.08) {
                  return 1;
                }
                else { // pl_ekf_ver > 1.08
                  return 3;
                }
              }
              else { // pl_ekf_hor > 3.18
                if (w_LLInum <= 30.5) {
                  return 1;
                }
                else { // w_LLInum > 30.5
                  return 4;
                }
              }
            }
            else { // w_noLLInum > 4.5
              if (f_avg_unc_pr <= 10.48) {
                if (pu_noLLIsatNum_G <= 5.5) {
                  return 1;
                }
                else { // pu_noLLIsatNum_G > 5.5
                  return 3;
                }
              }
              else { // f_avg_unc_pr > 10.48
                if (u_MeasTrackCount <= 48.5) {
                  return 3;
                }
                else { // u_MeasTrackCount > 48.5
                  return 4;
                }
              }
            }
          }
          else { // pu_CN0greater35NumPerSys_C > 12.5
            if (f_postCodeSTD <= 0.93) {
              if (f_max_CN0 <= 44.5) {
                if (pu_satNumPerSys_C <= 22.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 22.5
                  return 3;
                }
              }
              else { // f_max_CN0 > 44.5
                if (f_hdop <= 0.58) {
                  return 4;
                }
                else { // f_hdop > 0.58
                  return 1;
                }
              }
            }
            else { // f_postCodeSTD > 0.93
              if (pl_ekf_hor <= 1.76) {
                if (u_ppp_use_pr_num_float <= 14.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 14.5
                  return 1;
                }
              }
              else { // pl_ekf_hor > 1.76
                if (f_cmc_avg <= 0.52) {
                  return 3;
                }
                else { // f_cmc_avg > 0.52
                  return 1;
                }
              }
            }
          }
        }
      }
      else { // f_posLlaUnc_l > 0.94
        if (u_ppp_use_pr_num_float <= 9.5) {
          if (u_MeasTrackCount <= 36.5) {
            if (f_posLlaUnc_h <= 2.56) {
              if (u_satMeasCount <= 17.5) {
                if (w_cn0Num <= 20.5) {
                  return 4;
                }
                else { // w_cn0Num > 20.5
                  return 3;
                }
              }
              else { // u_satMeasCount > 17.5
                if (pu_satNumPerSys_C <= 15.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_C > 15.5
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 2.56
              if (f_pdop <= 60.97) {
                if (f_posLlaUnc_h <= 15.48) {
                  return 4;
                }
                else { // f_posLlaUnc_h > 15.48
                  return 3;
                }
              }
              else { // f_pdop > 60.97
                return 1;
              }
            }
          }
          else { // u_MeasTrackCount > 36.5
            if (pu_LLIsatNum_C <= 14.5) {
              if (pu_LLIsatNum_C <= 11.5) {
                if (f_cmc_std <= 5.39) {
                  return 4;
                }
                else { // f_cmc_std > 5.39
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 11.5
                if (f_posLlaUnc_h <= 1.98) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 1.98
                  return 4;
                }
              }
            }
            else { // pu_LLIsatNum_C > 14.5
              if (f_posLlaUnc_h <= 1.8) {
                if (pu_satNumPerSys_G <= 5.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_G > 5.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_h > 1.8
                return 4;
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 9.5
          if (pu_CN0greater35NumPerSys_C <= 11.5) {
            if (f_posLlaUnc_h <= 1.72) {
              if (w_LLInum <= 17.5) {
                return 1;
              }
              else { // w_LLInum > 17.5
                if (f_cn0_avg <= 36.75) {
                  return 3;
                }
                else { // f_cn0_avg > 36.75
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 1.72
              if (f_posLlaUnc_b <= 1.01) {
                if (pu_CN0greater35NumPerSys_C <= 2.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 2.5
                  return 4;
                }
              }
              else { // f_posLlaUnc_b > 1.01
                if (pu_CN0greater35NumPerSys_C <= 4.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 4.5
                  return 1;
                }
              }
            }
          }
          else { // pu_CN0greater35NumPerSys_C > 11.5
            if (f_hdop <= 0.82) {
              if (f_avg_unc_pr <= 5.32) {
                if (u_CN040 <= 23.5) {
                  return 3;
                }
                else { // u_CN040 > 23.5
                  return 4;
                }
              }
              else { // f_avg_unc_pr > 5.32
                if (f_postPhaseSTD <= 0.0) {
                  return 1;
                }
                else { // f_postPhaseSTD > 0.0
                  return 1;
                }
              }
            }
            else { // f_hdop > 0.82
              if (w_noLLInum <= 3.5) {
                if (f_postPhaseSTD <= 0.0) {
                  return 3;
                }
                else { // f_postPhaseSTD > 0.0
                  return 1;
                }
              }
              else { // w_noLLInum > 3.5
                if (f_posLlaUnc_h <= 9.1) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 9.1
                  return 4;
                }
              }
            }
          }
        }
      }
    }
  }
  else { // f_postCodeSTD > 1.12
    if (f_posLlaUnc_h <= 0.67) {
      if (u_CN040 <= 7.5) {
        if (w_noLLInum <= 16.5) {
          if (f_cn0_avg <= 28.75) {
            if (pu_noLLIsatNum_G <= 2.5) {
              if (f_avg_unc_pr <= 18.62) {
                if (f_posLlaUnc_b <= 0.37) {
                  return 1;
                }
                else { // f_posLlaUnc_b > 0.37
                  return 3;
                }
              }
              else { // f_avg_unc_pr > 18.62
                if (pu_satNumPerSys_G <= 7.5) {
                  return 1;
                }
                else { // pu_satNumPerSys_G > 7.5
                  return 3;
                }
              }
            }
            else { // pu_noLLIsatNum_G > 2.5
              if (pu_LLIsatNum_C <= 12.5) {
                if (pu_LLIsatNum_C <= 6.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_C > 6.5
                  return 3;
                }
              }
              else { // pu_LLIsatNum_C > 12.5
                if (f_cn0_std <= 8.53) {
                  return 3;
                }
                else { // f_cn0_std > 8.53
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_avg > 28.75
            if (pl_ekf_ver <= 0.57) {
              if (f_posLlaUnc_h <= 0.47) {
                if (f_cmc_avg <= 2.79) {
                  return 1;
                }
                else { // f_cmc_avg > 2.79
                  return 1;
                }
              }
              else { // f_posLlaUnc_h > 0.47
                if (f_posLlaUnc_b <= 0.15) {
                  return 2;
                }
                else { // f_posLlaUnc_b > 0.15
                  return 1;
                }
              }
            }
            else { // pl_ekf_ver > 0.57
              if (u_satMeasCount <= 24.5) {
                if (f_posLlaUnc_h <= 0.57) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.57
                  return 1;
                }
              }
              else { // u_satMeasCount > 24.5
                if (w_noLLInum <= 8.5) {
                  return 1;
                }
                else { // w_noLLInum > 8.5
                  return 3;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 16.5
          if (pu_noLLIsatNum_C <= 13.5) {
            if (f_posLlaUnc_h <= 0.43) {
              if (pu_CN0greater35NumPerSys_G <= 4.5) {
                if (pu_satNumPerSys_C <= 15.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_C > 15.5
                  return 3;
                }
              }
              else { // pu_CN0greater35NumPerSys_G > 4.5
                if (pu_satNumPerSys_G <= 8.5) {
                  return 2;
                }
                else { // pu_satNumPerSys_G > 8.5
                  return 1;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.43
              if (f_posLlaUnc_b <= 0.16) {
                if (w_LLInum <= 17.5) {
                  return 1;
                }
                else { // w_LLInum > 17.5
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 6.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 6.5
                  return 1;
                }
              }
            }
          }
          else { // pu_noLLIsatNum_C > 13.5
            if (f_posLlaUnc_b <= 0.11) {
              if (pu_CN0greater35NumPerSys_C <= 7.5) {
                if (u_CN040 <= 6.5) {
                  return 3;
                }
                else { // u_CN040 > 6.5
                  return 2;
                }
              }
              else { // pu_CN0greater35NumPerSys_C > 7.5
                if (u_MeasTrackCount <= 65.5) {
                  return 2;
                }
                else { // u_MeasTrackCount > 65.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_b > 0.11
              if (f_max_CN0 <= 48.5) {
                if (f_cn0_avg <= 30.5) {
                  return 3;
                }
                else { // f_cn0_avg > 30.5
                  return 1;
                }
              }
              else { // f_max_CN0 > 48.5
                if (f_vdop <= 1.73) {
                  return 4;
                }
                else { // f_vdop > 1.73
                  return 3;
                }
              }
            }
          }
        }
      }
      else { // u_CN040 > 7.5
        if (w_noLLInum <= 18.5) {
          if (pl_ekf_hor <= 1.24) {
            if (f_cmc_std <= 2.07) {
              if (f_cn0_avg <= 43.25) {
                if (w_LLInum <= 21.5) {
                  return 1;
                }
                else { // w_LLInum > 21.5
                  return 1;
                }
              }
              else { // f_cn0_avg > 43.25
                if (f_posLlaUnc_h <= 0.4) {
                  return 4;
                }
                else { // f_posLlaUnc_h > 0.4
                  return 3;
                }
              }
            }
            else { // f_cmc_std > 2.07
              if (pl_ekf_hor <= 0.77) {
                if (f_cmc_avg <= 6.28) {
                  return 1;
                }
                else { // f_cmc_avg > 6.28
                  return 2;
                }
              }
              else { // pl_ekf_hor > 0.77
                if (u_MeasTrackCount <= 65.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 65.5
                  return 3;
                }
              }
            }
          }
          else { // pl_ekf_hor > 1.24
            if (q_QRcheckStatus <= 0.5) {
              if (f_cn0_avg <= 29.25) {
                if (pl_ekf_ver <= 1.02) {
                  return 3;
                }
                else { // pl_ekf_ver > 1.02
                  return 1;
                }
              }
              else { // f_cn0_avg > 29.25
                if (w_noLLInum <= 11.5) {
                  return 1;
                }
                else { // w_noLLInum > 11.5
                  return 1;
                }
              }
            }
            else { // q_QRcheckStatus > 0.5
              if (pl_ekf_hor <= 1.51) {
                if (pl_ekf_ver <= 0.65) {
                  return 3;
                }
                else { // pl_ekf_ver > 0.65
                  return 1;
                }
              }
              else { // pl_ekf_hor > 1.51
                if (pu_satNumPerSys_G <= 8.5) {
                  return 3;
                }
                else { // pu_satNumPerSys_G > 8.5
                  return 1;
                }
              }
            }
          }
        }
        else { // w_noLLInum > 18.5
          if (f_cn0_avg <= 38.25) {
            if (f_posLlaUnc_h <= 0.18) {
              if (f_cn0_avg <= 34.75) {
                if (pl_ekf_ver <= 0.39) {
                  return 2;
                }
                else { // pl_ekf_ver > 0.39
                  return 3;
                }
              }
              else { // f_cn0_avg > 34.75
                if (u_CN040 <= 13.5) {
                  return 3;
                }
                else { // u_CN040 > 13.5
                  return 2;
                }
              }
            }
            else { // f_posLlaUnc_h > 0.18
              if (f_posLlaUnc_b <= 0.16) {
                if (f_postCodeSTD <= 2.81) {
                  return 1;
                }
                else { // f_postCodeSTD > 2.81
                  return 3;
                }
              }
              else { // f_posLlaUnc_b > 0.16
                if (pu_CN0greater35NumPerSys_C <= 8.5) {
                  return 4;
                }
                else { // pu_CN0greater35NumPerSys_C > 8.5
                  return 1;
                }
              }
            }
          }
          else { // f_cn0_avg > 38.25
            if (u_ppp_use_pr_num_float <= 34.5) {
              if (f_posLlaUnc_b <= 0.15) {
                if (f_postCodeSTD <= 1.73) {
                  return 1;
                }
                else { // f_postCodeSTD > 1.73
                  return 2;
                }
              }
              else { // f_posLlaUnc_b > 0.15
                if (u_ppp_use_pr_num_float <= 16.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 16.5
                  return 1;
                }
              }
            }
            else { // u_ppp_use_pr_num_float > 34.5
              if (u_MeasInUseCount <= 56.5) {
                if (f_gdop <= 1.25) {
                  return 2;
                }
                else { // f_gdop > 1.25
                  return 1;
                }
              }
              else { // u_MeasInUseCount > 56.5
                if (f_gdop <= 1.51) {
                  return 1;
                }
                else { // f_gdop > 1.51
                  return 2;
                }
              }
            }
          }
        }
      }
    }
    else { // f_posLlaUnc_h > 0.67
      if (pl_ekf_hor <= 3.15) {
        if (f_cn0_avg <= 30.75) {
          if (pu_satNumPerSys_C <= 14.5) {
            if (pl_ekf_hor <= 2.35) {
              if (u_satMeasCount <= 16.0) {
                if (f_posLlaUnc_h <= 0.72) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 0.72
                  return 1;
                }
              }
              else { // u_satMeasCount > 16.0
                if (f_posLlaUnc_b <= 0.38) {
                  return 4;
                }
                else { // f_posLlaUnc_b > 0.38
                  return 3;
                }
              }
            }
            else { // pl_ekf_hor > 2.35
              if (f_posLlaUnc_h <= 0.97) {
                if (f_max_CN0 <= 45.0) {
                  return 3;
                }
                else { // f_max_CN0 > 45.0
                  return 4;
                }
              }
              else { // f_posLlaUnc_h > 0.97
                if (f_cmc_avg <= 1.93) {
                  return 4;
                }
                else { // f_cmc_avg > 1.93
                  return 3;
                }
              }
            }
          }
          else { // pu_satNumPerSys_C > 14.5
            if (f_max_CN0 <= 46.5) {
              if (pl_ekf_ver <= 1.42) {
                if (pu_LLIsatNum_C <= 17.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_C > 17.5
                  return 1;
                }
              }
              else { // pl_ekf_ver > 1.42
                if (pu_LLIsatNum_G <= 9.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_G > 9.5
                  return 4;
                }
              }
            }
            else { // f_max_CN0 > 46.5
              if (u_MeasTrackCount <= 54.5) {
                if (f_posLlaUnc_h <= 1.35) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.35
                  return 3;
                }
              }
              else { // u_MeasTrackCount > 54.5
                if (u_CN040 <= 9.5) {
                  return 1;
                }
                else { // u_CN040 > 9.5
                  return 4;
                }
              }
            }
          }
        }
        else { // f_cn0_avg > 30.75
          if (pl_ekf_ver <= 1.26) {
            if (f_cn0_std <= 5.56) {
              if (u_MeasInUseCount <= 39.5) {
                if (pl_ekf_ver <= 1.11) {
                  return 1;
                }
                else { // pl_ekf_ver > 1.11
                  return 3;
                }
              }
              else { // u_MeasInUseCount > 39.5
                if (f_posLlaUnc_h <= 0.76) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.76
                  return 3;
                }
              }
            }
            else { // f_cn0_std > 5.56
              if (f_max_CN0 <= 48.5) {
                if (u_MeasInUseCount <= 44.5) {
                  return 1;
                }
                else { // u_MeasInUseCount > 44.5
                  return 3;
                }
              }
              else { // f_max_CN0 > 48.5
                if (f_posLlaUnc_h <= 0.77) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 0.77
                  return 1;
                }
              }
            }
          }
          else { // pl_ekf_ver > 1.26
            if (w_cn0Num <= 37.5) {
              if (f_cn0_avg <= 37.25) {
                if (w_LLInum <= 17.5) {
                  return 4;
                }
                else { // w_LLInum > 17.5
                  return 3;
                }
              }
              else { // f_cn0_avg > 37.25
                if (w_noLLInum <= 19.5) {
                  return 3;
                }
                else { // w_noLLInum > 19.5
                  return 3;
                }
              }
            }
            else { // w_cn0Num > 37.5
              if (pl_ekf_hor <= 2.26) {
                if (f_cmc_std <= 0.43) {
                  return 3;
                }
                else { // f_cmc_std > 0.43
                  return 3;
                }
              }
              else { // pl_ekf_hor > 2.26
                if (f_postCodeSTD <= 1.88) {
                  return 3;
                }
                else { // f_postCodeSTD > 1.88
                  return 4;
                }
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 3.15
        if (u_ppp_use_pr_num_float <= 12.5) {
          if (w_LLInum <= 12.5) {
            if (f_gdop <= 12.49) {
              if (pl_ekf_hor <= 4.24) {
                if (f_cn0_std <= 8.53) {
                  return 3;
                }
                else { // f_cn0_std > 8.53
                  return 4;
                }
              }
              else { // pl_ekf_hor > 4.24
                if (f_max_CN0 <= 35.5) {
                  return 1;
                }
                else { // f_max_CN0 > 35.5
                  return 4;
                }
              }
            }
            else { // f_gdop > 12.49
              if (pu_satNumPerSys_G <= 4.5) {
                return 4;
              }
              else { // pu_satNumPerSys_G > 4.5
                return 3;
              }
            }
          }
          else { // w_LLInum > 12.5
            if (f_max_CN0 <= 40.5) {
              if (pu_satNumPerSys_G <= 5.5) {
                if (f_posLlaUnc_h <= 1.8) {
                  return 1;
                }
                else { // f_posLlaUnc_h > 1.8
                  return 4;
                }
              }
              else { // pu_satNumPerSys_G > 5.5
                if (pl_ekf_ver <= 3.19) {
                  return 4;
                }
                else { // pl_ekf_ver > 3.19
                  return 4;
                }
              }
            }
            else { // f_max_CN0 > 40.5
              if (f_posLlaUnc_l <= 0.39) {
                if (f_posLlaUnc_h <= 1.01) {
                  return 3;
                }
                else { // f_posLlaUnc_h > 1.01
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.39
                if (f_posLlaUnc_b <= 0.35) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.35
                  return 4;
                }
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 12.5
          if (u_MeasTrackCount <= 35.5) {
            if (f_posLlaUnc_h <= 1.57) {
              if (f_posLlaUnc_l <= 0.64) {
                if (pu_CN0greater35NumPerSys_C <= 15.5) {
                  return 3;
                }
                else { // pu_CN0greater35NumPerSys_C > 15.5
                  return 1;
                }
              }
              else { // f_posLlaUnc_l > 0.64
                return 1;
              }
            }
            else { // f_posLlaUnc_h > 1.57
              if (u_MeasTrackCount <= 28.5) {
                return 1;
              }
              else { // u_MeasTrackCount > 28.5
                if (f_posLlaUnc_b <= 0.86) {
                  return 3;
                }
                else { // f_posLlaUnc_b > 0.86
                  return 1;
                }
              }
            }
          }
          else { // u_MeasTrackCount > 35.5
            if (f_posLlaUnc_h <= 1.37) {
              if (pu_CN0greater35NumPerSys_G <= 6.5) {
                if (pu_LLIsatNum_G <= 8.5) {
                  return 3;
                }
                else { // pu_LLIsatNum_G > 8.5
                  return 4;
                }
              }
              else { // pu_CN0greater35NumPerSys_G > 6.5
                if (u_MeasTrackCount <= 47.5) {
                  return 1;
                }
                else { // u_MeasTrackCount > 47.5
                  return 3;
                }
              }
            }
            else { // f_posLlaUnc_h > 1.37
              if (f_cn0_avg <= 38.75) {
                if (f_posLlaUnc_l <= 0.37) {
                  return 1;
                }
                else { // f_posLlaUnc_l > 0.37
                  return 4;
                }
              }
              else { // f_cn0_avg > 38.75
                if (u_ppp_use_pr_num_float <= 17.5) {
                  return 3;
                }
                else { // u_ppp_use_pr_num_float > 17.5
                  return 1;
                }
              }
            }
          }
        }
      }
    }
  }
}


/**
 * @brief determine scene type by Bagging DT model, max value from 4 trees model
 * @param[in]
 * @return
 */
uint8_t integ_DecisionTreePredictScene(integ_PppFeature_t* gpz_pppFeature) {

  uint8_t pd_predict_list[4] = { 0 };
  uint8_t  d_pred = 1;
  for (uint8_t u_i = 0; u_i < 4; u_i++) {
    pd_predict_list[u_i] = 1;
  }

  /* predict value from 10 trees */
  pd_predict_list[0] = predict_scene_0(gpz_pppFeature);
  pd_predict_list[1] = predict_scene_1(gpz_pppFeature);
  pd_predict_list[2] = predict_scene_2(gpz_pppFeature);
  pd_predict_list[3] = predict_scene_3(gpz_pppFeature);

  /* search the max value in pd_predict_list */
  for (uint8_t u_i = 0; u_i < 4; u_i++) {
    if (pd_predict_list[u_i] > d_pred) {
      d_pred = pd_predict_list[u_i];
    }
  }
  return d_pred;
}