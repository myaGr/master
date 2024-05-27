/**@file        rtk_integrity.c
 * @brief       Calculate PL for RTK FLOAT/FIX
 * @details     
 * @author      houxiaowei
 * @date        2023/5/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/25   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include <stdio.h>
#include <gnss_common.h>
#include "rtk_integrity.h"
#include "mw_alloc.h"
#include "gnss_identify_scene.h"
#include "cmn_utils.h"
#include "gnss_engine_api.h"
#include "rtk_type.h"
#include "gnss_filter_type.h"
#include "mw_log.h"
#include "rtk_integ_dt_model.h"

#define HIS_FAT_THRES   (uint8_t)(20)   /* history threshold epochs */

/**
 * @brief Set default integrity config
 * @param[in,out] pz_config
 */
static void rtk_integ_defaultConfig(integ_Config_t* pz_config)
{
  memset(pz_config, 0, sizeof(integ_Config_t));
  M_SET_BIT(pz_config->u_valid, C_INTEGRITY_LAUNCH_SET);
  M_SET_BIT(pz_config->u_valid, C_INTEGRITY_LAUNCH_INIT);
  pz_config->u_fun_type = C_INTEGRITY_FUN_DECISION_TREE;
}

extern integ_RtkIntegrity_t* rtk_integ_init()
{
  integ_RtkIntegrity_t* pz_integ = (integ_RtkIntegrity_t*)OS_MALLOC(sizeof(integ_RtkIntegrity_t));

  if (NULL != pz_integ)
  {
    rtk_integ_defaultConfig(&pz_integ->config);
  }
  return pz_integ;
}

extern void rtk_integ_deinit(integ_RtkIntegrity_t *pz_integ)
{
  OS_FREE(pz_integ);
}

/**
 * @brief Intergrity status
 * @param[in,out] pz_integ contains config and rtk factor
 * @return TRUE: ok, False: not ok
 */
extern int32_t rtk_integ_valid(const integ_RtkIntegrity_t *pz_integ)
{
  if ((pz_integ != NULL) &&
      (M_IS_SET_BIT(pz_integ->config.u_valid, C_INTEGRITY_LAUNCH_INIT)) &&
      (M_IS_SET_BIT(pz_integ->config.u_valid, C_INTEGRITY_LAUNCH_SET))
      )
  {
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief rtk integrity prepare work
 * @param[in,out] pz_integ RTK integrity interface struct
 * @param[in] pz_RTKfilterInfo EKF filter information
 * @param[in] pz_pvt_result  pack from PVT
 * @param[in] pz_satSigMeasCollect
 * @param[in] u_common_sat common satellites between rover station and base station
 */
extern void rtk_integ_start(integ_RtkIntegrity_t* pz_integ,
                            const rtk_filterInfo_t* pz_RTKfilterInfo,
                            const gnss_PVTResult_t* pz_pvt_result,
                            uint8_t u_common_sat)
{
  if(!rtk_integ_valid(pz_integ))
  {
    return;
  }
  /* Clear rtk feature */
  memset(&pz_integ->z_rtk_feature, 0, sizeof(integ_RtkFeature_t));

  /* Reset rtk feature */
  if ((pz_RTKfilterInfo->z_kfStatus == KF_INIT) || (pz_RTKfilterInfo->z_kfStatus == KF_RESTART_HPP))
  {
    memset(pz_integ, 0, sizeof(integ_RtkIntegrity_t));
    rtk_integ_defaultConfig(&pz_integ->config);
  }

  /*  Set common satellites between rover and base */
  pz_integ->z_rtk_feature.u_common_sat = u_common_sat;

  pz_integ->z_tor = pz_pvt_result->z_positionFix.z_gpsTime;
  pz_integ->z_rtk_feature.f_pdop = pz_pvt_result->z_positionFix.z_dops.f_pdop;
}

/**
 * @brief clear integrity information when RTK solution is invalid
 * @param[in,out] pz_integ RTK integrity interface struct
 */
extern void rtk_integ_clearInfo(integ_RtkIntegrity_t* pz_integ)
{
  pz_integ->u_sol_bit_count = 0;
  pz_integ->w_pre_sol_bit = 0;
}

/**
 * @brief Save omc residual
 * @param[in] z_residualType is the residual type, rtk_ResidualType
 * @param[in] d_lockTime carrier phase lock time
 * @param[in] d_all_cn0 sum of all signal cn0, contain cp and pr
 * @param[in] pu_resNumCodeAry array of different frequency code residual numbers
 * @param[in] pu_resNumPhaseAry array of different frequency phase residual numbers
 * @param[in] u_valid_sat array valid satellite
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_filterObs is the filter obs information
 * @param[in] pz_fixedAmbPool is the FIXED AMB, could be NULL in float case
 * @param[in,out] pz_integ contains config and rtk factor
 */
extern void rtk_integ_save_residualPost(rtk_ResidualType z_residualType, double d_lockTime, double d_all_cn0,
                                        const uint8_t *pu_resNumCodeAry, const uint8_t *pu_resNumPhaseAry,
                                        const uint8_t *u_valid_sat, const rtk_filterInfo_t *pz_RTKfilterInfo,
                                        const rtk_EpochFilterObs_t *pz_filterObs,
                                        const gnss_fixedSignalAmbPool_t *pz_fixedAmbPool,
                                        integ_RtkIntegrity_t *pz_integ)
{
  if (!rtk_integ_valid(pz_integ))
  {
    return;
  }

  if (any_Ptrs_Null(5, pu_resNumCodeAry, pu_resNumPhaseAry, u_valid_sat, pz_RTKfilterInfo, pz_filterObs))
  {
    return;
  }

  uint8_t u_resNumPhase = vector_u8_sum(pu_resNumPhaseAry, MAX_GNSS_SIGNAL_FREQ);
  uint8_t u_resNumCode = vector_u8_sum(pu_resNumCodeAry, MAX_GNSS_SIGNAL_FREQ);
  int32_t q_ns = vector_u8_sum(u_valid_sat, ALL_GNSS_SYS_SV_NUMBER);

  /* feature */
  integ_RtkFeature_t* pz_feat = &pz_integ->z_rtk_feature; 
 if (z_residualType == RES_POST)
  {
    pz_feat->f_code_uwrmse[1] = (float)pz_filterObs->d_postCodeSTD;
    pz_feat->f_phase_uwrmse[1] = (float)pz_filterObs->d_postPhaseSTD;
    pz_feat->u_code_number[1] = u_resNumCode;
    pz_feat->u_phase_number[1] = u_resNumPhase;
    pz_feat->u_sat_number[1] = q_ns;
    pz_feat->f_avg_cn0[1] = (float)d_all_cn0 / (float) (u_resNumPhase + u_resNumCode);
    pz_feat->f_avg_lock[1] = (float)d_lockTime / (float) u_resNumPhase;
    pz_feat->f_pdop_ekf_cp = (float)pz_filterObs->f_pdop;
  }
  else if (z_residualType == RES_FIXED && NULL != pz_fixedAmbPool)
  {
    pz_feat->f_code_uwrmse[0] = (float)pz_filterObs->d_fixedCodeSTD;
    pz_feat->f_phase_uwrmse[0] = (float)pz_filterObs->d_fixedPhaseSTD;
    pz_feat->u_code_number[0] = u_resNumCode;
    pz_feat->u_phase_number[0] = u_resNumPhase;
    pz_feat->u_sat_number[0] = q_ns;
    pz_feat->f_avg_cn0[0] = (float)d_all_cn0 / (float) (u_resNumPhase + u_resNumCode);
    pz_feat->f_avg_lock[0] = (float)d_lockTime / (float) u_resNumPhase;
    pz_feat->f_ratio = pz_fixedAmbPool->f_ratio;
    pz_feat->f_pdop_fix = pz_fixedAmbPool->f_pdop;
    pz_feat->f_adop_fix = pz_fixedAmbPool->f_adop;
    pz_feat->f_rss[0] = (float)M_MIN(999.9f, pz_fixedAmbPool->f_lambda_sigma[0]);
    pz_feat->f_rss[1] = (float)M_MIN(999.9f, pz_fixedAmbPool->f_lambda_sigma[1]);
  }
  else if (z_residualType == RES_PREFIX)
  {
    pz_feat->f_prefix_uwrmse = (float)pz_filterObs->d_prefixPhaseSTD;
  }
  else if(z_residualType == RES_RTD)
  {
    pz_feat->f_pdop_ekf_pr = (float)pz_filterObs->f_pdop;
  }
}

/**
 * @brief Save doppler omc residual
 * @param[in,out] pz_integ contains config and rtk factor
 * @param[in] d_sigma standard error of unit weight
 * @param[in] u_dr_num  numbers cont for doppler
 */
extern void rtk_integ_save_residualDr(double d_sigma, uint8_t u_dr_num, integ_RtkIntegrity_t* pz_integ)
{
  /* DR feature */
  pz_integ->z_rtk_feature.u_doppler_uwrmse = (float)d_sigma;
  pz_integ->z_rtk_feature.u_dr_number = u_dr_num;
}

/**
 * @brief rtk solution status of all types
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in,out] pz_integ contains config and rtk fact
 */
static void rtk_integ_solBit(const rtk_filterInfo_t* pz_RTKfilterInfo, integ_RtkIntegrity_t* pz_integ)
{
  uint16_t w_sol_bit = 0;
  rtk_filter_sol_t* pz_filterSol  = pz_RTKfilterInfo->pz_filterSol;

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_VALID)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_FIX);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] >= RTK_FILTER_SOL_STAT_VALID)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_PREFIX);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_VALID &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_WL] >= RTK_FILTER_SOL_STAT_VALID)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_FIX_WL);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_VALID &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_EWL] >= RTK_FILTER_SOL_STAT_VALID)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_FIX_EWL);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_FLOAT);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] >= RTK_FILTER_SOL_STAT_VALID)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_RTD);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_SPP] >= RTK_FILTER_SOL_STAT_SAVE)
  {
    M_SET_BIT(w_sol_bit, RTK_FILTER_POS_PREFIX);
  }

  if(w_sol_bit == pz_integ->w_pre_sol_bit)
  {
    pz_integ->u_sol_bit_count = (uint8_t)M_MIN(pz_integ->u_sol_bit_count++, HIS_FAT_THRES);
  }
  else
  {
    pz_integ->u_sol_bit_count = 0;
  }
  pz_integ->z_rtk_feature.w_sol_bit = w_sol_bit;
  pz_integ->w_pre_sol_bit = w_sol_bit;
}

/**
 * @brief save position engine status, Eg. scene, solution tag, KF count...
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_posSolution poisition struct
 * @param[in,out] pz_integ contains config and rtk factor
 * @return FALSE: filter solution invalid, TRUE: success
 */
extern void rtk_integ_save_statusInfo(const rtk_filterInfo_t* pz_RTKfilterInfo,
                                      const gnss_PositionFix_t* pz_posSolution,
                                      integ_RtkIntegrity_t* pz_integ)
{
  integ_RtkFeature_t* pz_feat = &pz_integ->z_rtk_feature;
  pz_feat->u_fixFlag = pz_posSolution->u_fixFlag;
  pz_feat->u_solTag = pz_RTKfilterInfo->u_solTag;

  /* u_sol_bit_count, solution bit  */
  rtk_integ_solBit(pz_RTKfilterInfo, pz_integ);

  /* KF count */
  pz_integ->u_ekf_count = M_MIN(pz_integ->u_ekf_count++, (uint8_t)(UINT8_MAX-1));

  /* scene*/
  pz_feat->u_scene = pz_RTKfilterInfo->u_goodSceneStatusFlag;
  pz_integ->t_seq_open_sky = pz_RTKfilterInfo->t_openSkyCount;
  pz_integ->t_seq_close_sky = pz_RTKfilterInfo->t_closeSkyCount;
  pz_feat->u_opensky_count = sum_of_bit(pz_integ->t_seq_open_sky, HIS_FAT_THRES);
  pz_feat->u_closesky_count = sum_of_bit(pz_integ->t_seq_close_sky, HIS_FAT_THRES);

  /* u_fix_count, GNSS_FILTER_POS_FIX conut, */
  if (pz_feat->u_solTag == GNSS_FILTER_POS_FIX)
  {
    pz_integ->u_fix_count = M_MIN(pz_integ->u_fix_count++, HIS_FAT_THRES);
  }
  else
  {
    pz_integ->u_fix_count = 0;
  }
}

/**
 * @brief save WL feature information
 * @param[in] pz_fixedAmbPool
 * @param[in] u_rtkFilterPosStatus
 * @param[in,out] pz_rtkAmbFixInputInfo
 */
extern void rtk_integ_save_wideLaneInfo(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                                        uint8_t u_rtkFilterPosStatus,
                                        gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo)
{
  integ_RtkWLFeature_t* pz_wlfeat = &pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pz_integ->z_rtk_feature.z_wl[u_rtkFilterPosStatus];
  pz_wlfeat->u_status = 1;
  pz_wlfeat->u_fix_sat_number = pz_fixedAmbPool->u_nb;
  pz_wlfeat->u_avg_lock = 0;
  pz_wlfeat->u_ref_lock = 0;
  pz_wlfeat->f_avg_cn0 = 0;
  pz_wlfeat->f_ratio = pz_fixedAmbPool->f_ratio;
  pz_wlfeat->f_rss[0] = (float)M_MIN(999.9, pz_fixedAmbPool->f_lambda_sigma[0]);
  pz_wlfeat->f_rss[1] = (float)M_MIN(999.9, pz_fixedAmbPool->f_lambda_sigma[1]);
  pz_wlfeat->f_adop = pz_fixedAmbPool->f_adop;
  pz_wlfeat->f_pdop = pz_fixedAmbPool->f_pdop;
  pz_wlfeat->f_uwrmse = 0.0f;
}

/**
 * @brief Extract STD from EKF filter P matrix
 * @param[in] pz_EKFstateRepPool
 * @param[in] pz_posSolution
 * @param[in,out] pz_RTKfilterInfo
 */
extern void rtk_integ_save_KFSTD(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
                                 const gnss_PositionFix_t* pz_posSolution,
                                 rtk_filterInfo_t* pz_RTKfilterInfo)
{
  integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;
  float f_factor = 4.42f;
  double* d_QEcefPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  double* d_QenuPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  int16_t pw_PVAindex[PVA_NUM] = {0};

  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);

  if (getPVAQ(pz_RTKfilterInfo->pd_Q, pw_PVAindex, 3, d_QEcefPos))
  {
    gnss_CovEcef2Enu(pz_posSolution->d_lla, d_QEcefPos, d_QenuPos);
    pz_feat->f_kf_std[0] = (float)sqrt(d_QenuPos[0] + d_QenuPos[4]) * f_factor;
    pz_feat->f_kf_std[1] = (float)sqrt(d_QenuPos[8]) * f_factor;
  }
  OS_FREE(d_QEcefPos);
  OS_FREE(d_QenuPos);
}

/**
 * @brief Log feature
 * @param[in] pz_RTKfilterInfo
 */
extern void rkt_integ_logFeature(const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }

  EpochTime_t z_epochT = {0};
  char u_buffer[BUFF_SIZE] = {0};
  char* p = u_buffer;
  char u_buffer_header[BUFF_SIZE] = {0};
  char* p1 = u_buffer_header;

  integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  tm_cvt_GpstToEpoch(&pz_integ->z_tor, &z_epochT);

  integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;
  p += sprintf(p, "%6.2u, ", pz_integ->z_tor.w_week);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "w_week,");
  p += sprintf(p, "%6.2f, ", pz_integ->z_tor.q_towMsec * TIME_MSEC_INV);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "q_towMsec,");
  p += sprintf(p, "%4d/%02d/%02d %02d:%02d:%05.2f, ", z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour, z_epochT.min, z_epochT.second);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "utc_time,");

  p += sprintf(p, "%6.2u, ", pz_feat->u_fixFlag);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_fixFlag,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_solTag);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_solTag,");
  p += sprintf(p, "%6.2u, ", pz_feat->w_sol_bit);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "w_sol_bit,");

  p += sprintf(p, "%6.2u, ", pz_integ->u_fix_count);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_fix_count,");
  p += sprintf(p, "%6.2u, ", pz_integ->u_sol_bit_count);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_sol_bit_count,");
  p += sprintf(p, "%6.2u, ", pz_integ->u_ekf_count);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_ekf_count,");

  p += sprintf(p, "%6.2u, ", pz_feat->u_common_sat);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_common_sat,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_scene);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_scene,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_cmc);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_cmc,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_pdop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_pdop,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_pdop_ekf_pr);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_pdop_ekf_pr,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_pdop_ekf_cp);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_pdop_ekf_cp,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_opensky_count);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_opensky_count,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_closesky_count);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_closesky_count,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_dr_number);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_dr_number,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_sat_number[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_sat_number_0,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_sat_number[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_sat_number_1,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_phase_number[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_phase_number_0,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_phase_number[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_phase_number_1,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_code_number[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_code_number_0,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_code_number[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_code_number_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_phase_uwrmse[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_phase_uwrmse_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_phase_uwrmse[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_phase_uwrmse_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_code_uwrmse[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_code_uwrmse_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_code_uwrmse[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_code_uwrmse_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_avg_lock[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_avg_lock_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_avg_lock[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_avg_lock_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_avg_cn0[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_avg_cn0_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_avg_cn0[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_avg_cn0_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_ratio);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_ratio,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_adop_fix);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_adop_fix,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_pdop_fix);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_pdop_fix,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_rss[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_rss_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_rss[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_rss_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_kf_std[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_kf_std_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_kf_std[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_kf_std_1,");

  /* WL */
  p += sprintf(p, "%6.2u, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].u_fix_sat_number);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","u_fix_sat_number,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_ratio);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","f_ratio,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_rss[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","f_rss_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_rss[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","f_rss_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_adop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","f_adop,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_pdop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","wl_","f_pdop,");

  /* EWL */
  p += sprintf(p, "%6.2u, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].u_fix_sat_number);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","u_fix_sat_number,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_ratio);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","f_ratio,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_rss[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","f_rss_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_rss[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","f_rss_1,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_adop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","f_adop,");
  p += sprintf(p, "%6.2f, ", pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_pdop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "%s%s","ewl_","f_pdop,");
  /* prefix */
  p += sprintf(p, "%6.2f, ", pz_feat->f_previous_ratio);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_previous_ratio,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_previous_fix_sat);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_previous_fix_sat,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_prefix_age);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_prefix_age,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_prefix_uwrmse);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_prefix_uwrmse,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_prefix_ls_std[0]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_prefix_ls_std_0,");
  p += sprintf(p, "%6.2f, ", pz_feat->f_prefix_ls_std[1]);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_prefix_ls_std_1,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_prefix_sat_number);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_prefix_sat_number,");
  p += sprintf(p, "%6.2u, ", pz_feat->u_prefix_sig_number);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "u_prefix_sig_number,");

  p += sprintf(p, "%6.2f", pz_feat->f_prefix_pdop);
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "f_prefix_pdop");

  p += sprintf(p, "\n");
  p1 += log_DataLogHeaderSprintf(DLOG_GNSS_ML_PL, p1, "\n");

  DATA_LOG_HEADER(DLOG_GNSS_ML_PL, "%s", u_buffer_header);
  DATA_LOG(DLOG_GNSS_ML_PL, "%s", u_buffer);
}

/**
 * @brief Calculate STD by Decision Tree
 * @param[in] pz_RTKfilterInfo RTK filter information
 * @param[in,out] pz_posSolution position solution of RTK
 */
void rtk_integ_calSTDByDecisionTree(const rtk_filterInfo_t* pz_RTKfilterInfo, gnss_PositionFix_t* pz_posSolution)
{
  double d_posLlaUnc[3] = {0.0};
  double d_posXyzUnc[3] = {0.0};

  float f_std = rtk_dt_STDPredict(pz_RTKfilterInfo);

  if(f_std != 99.9f)
  {
    /* LLA uncertain, lla */
    pz_posSolution->f_posLlaUnc[0] = f_std / 1.414f;
    pz_posSolution->f_posLlaUnc[1] = f_std / 1.414f;
    pz_posSolution->f_posLlaUnc[2] = f_std / 1.414f * 2.0f;

    /* Position uncertain, xyz */
    d_posLlaUnc[0] = (double)pz_posSolution->f_posLlaUnc[0];
    d_posLlaUnc[1] = (double)pz_posSolution->f_posLlaUnc[1];
    d_posLlaUnc[2] = (double)pz_posSolution->f_posLlaUnc[2];
    gnss_Enu2Ecef(pz_posSolution->d_lla, d_posLlaUnc, d_posXyzUnc);
    pz_posSolution->f_posXyzUnc[0] = (float)d_posXyzUnc[0];
    pz_posSolution->f_posXyzUnc[1] = (float)d_posXyzUnc[1];
    pz_posSolution->f_posXyzUnc[2] = (float)d_posXyzUnc[2];
  }
}
