/**@file        service ssr_cvt.c
 * @brief       service ssr data convert functions
 *              1. QianXun SSR data struct to Asensing SSR struct
 *              2. GeeSpace SSR data struct to Asensing SSR struct
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/29  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "service_ssr_cvt.h"
#include "gnss_common.h"
#include "cmn_utils.h"

 /**
  * @brief Convert Time structure from QXWZ to Asensing
  * @param[in] pz_qx_gps_time - Qianxun's time data
  * @param[in] pz_ag_gps_time - Asensing's time data
  * @return    None
  */
static void ssr_cvt_Time_Qxwz2Ag(const qxsi_gps_time_t* pz_qx_gps_time, GpsTime_t* pz_ag_gps_time)
{
  if ((NULL == pz_qx_gps_time) ||
     (NULL == pz_ag_gps_time))
  {
    return;
  }

  tm_cvt_SetGpst(pz_ag_gps_time, pz_qx_gps_time->week_num, pz_qx_gps_time->sec_of_week);
  return;
}

/**
 * @brief Convert Time structure from Dayou to Asensing
 * @param[in] pz_dy_gps_time - Dayou time data
 * @param[in] pz_ag_gps_time - Asensing's time data
 * @return    None
 */
static void ssr_cvt_Time_Dysk2Ag(const Gtime_t* pz_dy_gps_time, GpsTime_t* pz_ag_gps_time)
{
  uint16_t week = 0;
  double tow = 0.0;
  if ((NULL == pz_dy_gps_time) ||
    (NULL == pz_ag_gps_time))
  {
    return;
  }
  tm_cvt_FullMsecToGpsWeekTow((uint32_t)pz_dy_gps_time->time, pz_dy_gps_time->sec, &week, &tow);
  tm_cvt_SetGpst(pz_ag_gps_time, week, tow);
  return;
}

/**
 * @brief Convert atmo mask from QXWZ to Asensing
 * @param[in] u_qx_atmo_mask - Qianxun's atmo mask
 * @return    Asensing's atmo mask
 */
static gnss_ssrAtmoMask ssr_cvt_AtmoMask_Qxwz2Ag(qxsi_ssr_atmo_mask u_qx_atmo_mask)
{
  gnss_ssrAtmoMask u_ag_atmo_mask = 0;
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_STEC_CORR)
  {
    u_ag_atmo_mask |= GNSS_SSR_ATMO_STEC_CORR;
  }
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_STD_CORR)
  {
    u_ag_atmo_mask |= GNSS_SSR_ATMO_STD_CORR;
  }
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_ZTD_CORR)
  {
    u_ag_atmo_mask |= GNSS_SSR_ATMO_ZTD_CORR;
  }
  return u_ag_atmo_mask;
}

/**
 * @brief Convert error model mask from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's error model mask
 * @return    Asensing's error model mask
 */
static gnss_ssrErrorModelMask ssr_cvt_ErrorModelMask_Qxwz2Ag(qxsi_ssr_site_error_model_mask u_qx_error_mask)
{
  gnss_ssrErrorModelMask u_ag_error_mask = 0;
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_SOLID_TIDE_CORR)
  {
    u_ag_error_mask |= GNSS_SSR_ERROR_MODEL_SOLID_TIDE_CORR;
  }
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_OCEAN_TIDE_CORR)
  {
    u_ag_error_mask |= GNSS_SSR_ERROR_MODEL_OCEAN_TID_CORR;
  }
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_POLE_TIDE_CORR)
  {
    u_ag_error_mask |= GNSS_SSR_ERROR_MODEL_POLE_TIDE_CORR;
  }
  return u_ag_error_mask;
}

/**
 * @brief Convert sat error model mask from Qxsi to Asensing
 * @param[in] u_qx_sat_error_mask - Qianxun's sat error model mask
 * @return    Asensing's sat error model mask
 */
static gnss_ssrSatErrorModelMask ssr_cvt_SatErrorModelMask_Qxwz2Ag(qxsi_ssr_sat_error_model_mask u_qx_sat_error_mask)
{
  gnss_ssrErrorModelMask u_ag_sat_error_mask = 0;
  if (u_qx_sat_error_mask & QXSI_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR)
  {
    u_ag_sat_error_mask |= QXSI_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR;
  }
  if (u_qx_sat_error_mask & QXSI_SSR_ERROR_MODEL_SAT_BDS_MULTIPATH_CORR)
  {
    u_ag_sat_error_mask |= QXSI_SSR_ERROR_MODEL_SAT_BDS_MULTIPATH_CORR;
  }
  if (u_qx_sat_error_mask & QXSI_SSR_ERROR_MODEL_SAT_WIND_UP_VALID)
  {
    u_ag_sat_error_mask |= QXSI_SSR_ERROR_MODEL_SAT_WIND_UP_VALID;
  }
  if (u_qx_sat_error_mask & QXSI_SSR_ERROR_MODEL_SAT_YAW_INFO_VALID)
  {
    u_ag_sat_error_mask |= QXSI_SSR_ERROR_MODEL_SAT_YAW_INFO_VALID;
  }
  return u_ag_sat_error_mask;
}

/**
 * @brief Convert orbit clock mask from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's orbit clock mask
 * @return    Asensing's orbit clock mask
 */
static gnss_ssrSatOrbClkMask ssr_cvt_OrbClkMask_Qxwz2Ag(qxsi_ssr_sat_orb_clk_mask u_qx_orb_clk_mask)
{
  gnss_ssrSatOrbClkMask u_ag_orb_clk_mask = 0;
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_EPH)
  {
    u_ag_orb_clk_mask |= GNSS_SSR_SAT_ORB_CLK_EPH;
  }
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_ORBIT_CORR)
  {
    u_ag_orb_clk_mask |= GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR;
  }
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_CLOCK_CORR)
  {
    u_ag_orb_clk_mask |= GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR;
  }
  return u_ag_orb_clk_mask;
}

/**
 * @brief Convert Satellites bias mask from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's Satellites bias mask
 * @return    Asensing's Satellites bias mask
 */
static gnss_ssrSatBiasMask ssr_cvt_SatBiasMask_Qxwz2Ag(qxsi_ssr_sat_bias_mask u_qx_sat_bias_mask)
{
  gnss_ssrSatBiasMask u_ag_sat_bias_mask = 0;
  if (u_qx_sat_bias_mask & QXSI_SSR_SAT_BIAS_CODE_CORR)
  {
    u_ag_sat_bias_mask |= GNSS_SSR_SAT_BIAS_CODE_CORR;
  }
  if (u_qx_sat_bias_mask & QXSI_SSR_SAT_BIAS_PHASE_CORR)
  {
    u_ag_sat_bias_mask |= GNSS_SSR_SAT_BIAS_PHASE_CORR;
  }
  return u_ag_sat_bias_mask;
}

/**
 * @brief Convert integrity flag from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's integrity flag
 * @return    Asensing's integrity flag
 */
static gnss_integrityFlag ssr_cvt_IntegrityFlag_Qxwz2Ag(qxsi_integrity_flag u_qx_integrity_flag)
{
  gnss_integrityFlag u_ag_integrity_flag = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  if (QXSI_INTEGRITY_FLAG_MONITORED_OK == u_qx_integrity_flag)
  {
    u_ag_integrity_flag = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  }
  else if (QXSI_INTEGRITY_FLAG_MONITORED_FAIL == u_qx_integrity_flag)
  {
    u_ag_integrity_flag = GNSS_INTEGRITY_FLAG_MONITORED_FAIL;
  }
  else if (QXSI_INTEGRITY_FLAG_NOT_MONITORED == u_qx_integrity_flag)
  {
    u_ag_integrity_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  }
  return u_ag_integrity_flag;
}

/**
 * @brief Convert constellation flag from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's constellation flag
 * @return    Asensing's constellation flag
 */
static uint8_t ssr_cvt_cnstl_Qxwz2Ag(qxsi_sat_sys_e qx_cnstl, int prn)
{
  uint8_t u_ag_cnstl = C_GNSS_MAX;
  if (QXSI_SAT_SYS_GPS == qx_cnstl)
  {
    u_ag_cnstl = C_GNSS_GPS;
  }
  else if (QXSI_SAT_SYS_BDS == qx_cnstl)
  {
    if (prn > 18)
    {
      u_ag_cnstl = C_GNSS_BDS3;
    }
    else
    {
      u_ag_cnstl = C_GNSS_BDS2;
    }
  }
  else if (QXSI_SAT_SYS_GAL == qx_cnstl)
  {
    u_ag_cnstl = C_GNSS_GAL;
  }
  return u_ag_cnstl;
}

/**
 * @brief Convert constellation flag from Dayou to Asensing
 * @param[in] u_qx_error_mask - Dayou's constellation flag
 * @return    Asensing's constellation flag
 */
static uint8_t ssr_cvt_cnstl_Dysk2Ag(int32_t dy_cnstl,int prn)
{
  uint8_t u_ag_cnstl = C_GNSS_MAX;

  switch (dy_cnstl)
  {
  case DY_SYS_GPS:u_ag_cnstl = C_GNSS_GPS; break;
  case DY_SYS_GAL:u_ag_cnstl = C_GNSS_GAL; break;
  case DY_SYS_CMP:
  {
    if (prn >= 18)
    {
      u_ag_cnstl = C_GNSS_BDS3;
    }
    else
    {
      u_ag_cnstl = C_GNSS_BDS2;
    }
    break;
  }
  default:
    u_ag_cnstl = C_GNSS_MAX; break;
  }
  
  return u_ag_cnstl;
}

/**
 * @brief Convert correction bias from Qxsi to Asensing
 * @param[in] p_qxsiSsrBias - Qianxun's correction bias
 * @param[out] pz_agSsrCorrBias - Asensing's correction bias
 * @return    None
 */
static void ssr_cvt_CorrBias_Qxwz2Ag(const qxsi_corr_data_t* pz_qxsiSsrBias, gnss_SsrCorrBias_t* pz_agSsrCorrBias)
{
  if (pz_qxsiSsrBias && pz_agSsrCorrBias)
  {
    pz_agSsrCorrBias->f_qi = pz_qxsiSsrBias->qi;
    pz_agSsrCorrBias->q_corr = (int32_t)(pz_qxsiSsrBias->corr * 1000.0 + 0.5);
    pz_agSsrCorrBias->u_postItg = ssr_cvt_IntegrityFlag_Qxwz2Ag(pz_qxsiSsrBias->post_itg);
    pz_agSsrCorrBias->u_preItg = ssr_cvt_IntegrityFlag_Qxwz2Ag(pz_qxsiSsrBias->pre_itg);
  }
  return;
}

/**
 * @brief Get Signal type from Qianxun sys and signal
 * @param[in] sat_sys - Qianxun's correction bias
 * @param[in] qx_signal - Qianxun's correction bias
 * @return    gnss signal type
 */
gnss_SignalType ssr_cvt_QxwzSysCh2SignalType(qxsi_sat_sys_e qxsi_sat_sys,
  const qxsi_channel_t* pz_qxsi_signal_ch)
{
  gnss_SignalType u_signalType = C_GNSS_SIG_MAX;
  if (QXSI_SAT_SYS_GPS == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GPS_L1C;
      }
    }
    else if (2 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GPS_L2C;
      }
      if (CHANNEL_IDX_L == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GPS_L2C;
      }
      if (CHANNEL_IDX_W == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GPS_L2C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_Q == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GPS_L5Q;
      }
    }
  }
#if 0
  else if (QXSI_SAT_SYS_GLO == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GLO_G1;
      }
    }
    else if (2 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_GLO_G2;
      }
    }
  }
#endif
  else if (QXSI_SAT_SYS_BDS == qxsi_sat_sys)
  {
    if (2 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_BDS_B1I;
      }
    }
    else if (7 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_BDS_B2I;
      }
    }
    else if (6 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_BDS_B3I;
      }
    }
    else if (1 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_P == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_BDS_B1C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_P == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_BDS_B2A;
      }
    }
  }
  else if (QXSI_SAT_SYS_GAL == qxsi_sat_sys)//to be improved
  {
    if (1 == (pz_qxsi_signal_ch->frq))
    {
      u_signalType = C_GNSS_SIG_GAL_E1;
    }
    else if (7 == (pz_qxsi_signal_ch->frq))
    {
      u_signalType = C_GNSS_SIG_GAL_E5B;
    }
    else if (5 == (pz_qxsi_signal_ch->frq))
    {
      u_signalType = C_GNSS_SIG_GAL_E5A;
    }
  }
#if 0
  else if (QXSI_SAT_SYS_QZS == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_QZS_L1C;
      }
    }
    else if (2 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_QZS_L2C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch->frq))
    {
      if (CHANNEL_IDX_Q == (pz_qxsi_signal_ch->tag))
      {
        u_signalType = C_GNSS_SIG_QZS_L5Q;
      }
    }
  }
#endif
  return u_signalType;
}

/**
 * @brief Get Signal type from dysk sys and signal
 * @param[in] w_sys - dysk's correction bias
 * @param[in] u_code - dysk's correction bias
 * @return    gnss signal type
 */
gnss_SignalType ssr_cvt_DyskSysCh2SignalType(int32_t w_sys,uint8_t u_code)
{
  gnss_SignalType u_signalType = C_GNSS_SIG_MAX;
  if (DY_SYS_GPS == w_sys)
  {
    if (1 == u_code)
    {
      u_signalType = C_GNSS_SIG_GPS_L1C;
    }
    else if (14 == u_code|| 17 == u_code)
    {
      u_signalType = C_GNSS_SIG_GPS_L2C;
    }
    else if (25 == u_code)
    {
      u_signalType = C_GNSS_SIG_GPS_L5Q;
    }
  }
  else if (DY_SYS_CMP == w_sys)
  {
    if (40 == u_code)
    {
      u_signalType = C_GNSS_SIG_BDS_B1I;
    }
    else if (27 == u_code)
    {
      u_signalType = C_GNSS_SIG_BDS_B2I;
    }
    else if (42 == u_code)
    {
      u_signalType = C_GNSS_SIG_BDS_B3I;
    }
    else if (2 == u_code)
    {
      u_signalType = C_GNSS_SIG_BDS_B1C;
    }
    else if (58 == u_code)
    {
      u_signalType = C_GNSS_SIG_BDS_B2A;
    }
  }
  else if (DY_SYS_GAL == w_sys)
  {
    if (13 >= u_code)
    {
      u_signalType = C_GNSS_SIG_GAL_E1;
    }
    else if (26 >= u_code)
    {
      u_signalType = C_GNSS_SIG_GAL_E5A;
    }
    else if (29 >= u_code)
    {
      u_signalType = C_GNSS_SIG_GAL_E5B;
    }
  }
  return u_signalType;
}


/**
 * @brief Get Signal type from Qianxun sys and signal
 * @param[in] sat_sys - Qianxun's correction bias
 * @param[in] qx_signal - Qianxun's correction bias
 * @return    GNSS Signal type
 */
uint8_t ssr_cvt_SatSsrLos_Qxwz2Ag(const qxsi_sat_los_t* p_qxSatLos, gnss_satSsrLos_t* p_algoSatLos)
{
  uint8_t u_validLos = 0, u_ichan = 0, u_algoChanNum = 0;
  gnss_SignalType u_algoSignal = C_GNSS_SIG_MAX;
  p_algoSatLos->u_orbClkMask = ssr_cvt_OrbClkMask_Qxwz2Ag(p_qxSatLos->orb_clk_mask);
  p_algoSatLos->u_atmoMask = ssr_cvt_AtmoMask_Qxwz2Ag(p_qxSatLos->atmo_mask);
  p_algoSatLos->u_windUpMask = ssr_cvt_SatErrorModelMask_Qxwz2Ag(p_qxSatLos->sat_error_model_mask);
  p_algoSatLos->q_iode = (int32_t)p_qxSatLos->brdc_info.iode;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[0] = p_qxSatLos->brdc_info.pos.x;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[1] = p_qxSatLos->brdc_info.pos.y;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[2] = p_qxSatLos->brdc_info.pos.z;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[3] = p_qxSatLos->brdc_info.clock;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[0] = p_qxSatLos->brdc_info.vel.x;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[1] = p_qxSatLos->brdc_info.vel.y;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[2] = p_qxSatLos->brdc_info.vel.z;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[3] = p_qxSatLos->brdc_info.clock_drift;
  ssr_cvt_CorrBias_Qxwz2Ag(&(p_qxSatLos->orb_clk), &(p_algoSatLos->z_orbClk));
  ssr_cvt_Time_Qxwz2Ag(&(p_qxSatLos->orb_clk_time), &(p_algoSatLos->z_orbClkTime));
  ssr_cvt_CorrBias_Qxwz2Ag(&(p_qxSatLos->stec), &(p_algoSatLos->z_stec));
  ssr_cvt_CorrBias_Qxwz2Ag(&(p_qxSatLos->std), &(p_algoSatLos->z_std));
  p_algoSatLos->d_windUp = p_qxSatLos->windup;
  p_algoSatLos->f_yaw = p_qxSatLos->yaw;
  p_algoSatLos->f_yawRate = p_qxSatLos->yaw_rate;
  for (u_ichan = 0; u_ichan < (p_qxSatLos->signal_num); ++u_ichan)
  {
    if (u_algoChanNum >= M_ARRAY_SIZE(p_algoSatLos->z_signalBiasCorr))
    {
      break;
    }
    u_algoSignal = ssr_cvt_QxwzSysCh2SignalType(p_qxSatLos->id.sys, &(p_qxSatLos->signal_bias[u_ichan].signal_type));
    if (C_GNSS_SIG_MAX == u_algoSignal)
    {
      continue;
    }
    p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_signalType = u_algoSignal;
    p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_discontinuityIod = (uint8_t)p_qxSatLos->signal_bias[u_ichan].phase_bias_continue_flag;
    p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_biasMask = ssr_cvt_SatBiasMask_Qxwz2Ag(p_qxSatLos->signal_bias[u_ichan].bias_mask);
    ssr_cvt_CorrBias_Qxwz2Ag(&(p_qxSatLos->signal_bias[u_ichan].P), &(p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_codeBias));
    ssr_cvt_CorrBias_Qxwz2Ag(&(p_qxSatLos->signal_bias[u_ichan].L), &(p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_phaseBias));
    ++u_algoChanNum;
  }
  if (u_algoChanNum > 0)
  {
    u_validLos = 1;
  }
  p_algoSatLos->u_signalNum = u_algoChanNum;
  return u_validLos;
}

/**
 * @brief Convert correction from dayou to asensing
 * @param[in] p_dySatLos - dayou's correction
 * @param[in] p_algoSatLos - asensing's correction
 * @return    flag,0:unhealth, other:health
 */
uint8_t ssr_cvt_SatSsrLos_Dysk2Ag(const DySatLos_t* p_dySatLos, gnss_satSsrLos_t* p_algoSatLos)
{
  double d_tecu_fact = 1.0;
  uint8_t u_validLos = 0, u_ichan = 0, u_algoChanNum = 0;
  gnss_SignalType u_algoSignal = C_GNSS_SIG_MAX;
  
  p_algoSatLos->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_EPH;
  p_algoSatLos->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR;
  p_algoSatLos->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR;
  p_algoSatLos->z_orbClk.f_qi = 0.0;
  p_algoSatLos->z_orbClk.q_corr = (int32_t)(p_dySatLos->OrtClk*1000+0.5);
  p_algoSatLos->z_orbClk.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  p_algoSatLos->z_orbClk.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;

  if (fabs(p_dySatLos->Stec[0] > 0) || fabs(p_dySatLos->Stec_std[0]) > 0)
  {
    p_algoSatLos->u_atmoMask |= GNSS_SSR_ATMO_STEC_CORR;
    d_tecu_fact = 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(ssr_cvt_cnstl_Dysk2Ag(p_dySatLos->sys, p_dySatLos->prn)));
    p_algoSatLos->z_stec.f_qi = (float)(p_dySatLos->Stec_std[0]/ d_tecu_fact);
    p_algoSatLos->z_stec.q_corr = -(int32_t)(1000 * p_dySatLos->Stec[0] / d_tecu_fact+0.5);
    p_algoSatLos->z_stec.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
    p_algoSatLos->z_stec.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  }
  if (fabs(p_dySatLos->Trp) > 0)
  {
    p_algoSatLos->u_atmoMask |= GNSS_SSR_ATMO_STD_CORR;
    p_algoSatLos->z_std.f_qi = 0.0;
    p_algoSatLos->z_std.q_corr = (int32_t)(p_dySatLos->Trp*1000.0+0.5);
    p_algoSatLos->z_std.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
    p_algoSatLos->z_std.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  }
  p_algoSatLos->u_windUpMask = 0;
  p_algoSatLos->q_iode = (int32_t)p_dySatLos->iode;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[0] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[1] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[2] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satPosClk[3] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[0] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[1] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[2] = 0.0;
  p_algoSatLos->z_satPosVelClkBrdc.d_satVelClk[3] = 0.0;
  ssr_cvt_Time_Dysk2Ag(&(p_dySatLos->SsrTime), &(p_algoSatLos->z_orbClkTime));
  p_algoSatLos->d_windUp = p_dySatLos->Phw[0] * gnss_BaseL1Freq(ssr_cvt_cnstl_Dysk2Ag(p_dySatLos->sys, p_dySatLos->prn)) / CLIGHT;
  p_algoSatLos->f_yaw = 0.0;
  p_algoSatLos->f_yawRate = 0.0;
  for (u_ichan = 0; u_ichan < (DY_NFREQ + DY_NEXOBS); ++u_ichan)
  {
    if (u_algoChanNum >= M_ARRAY_SIZE(p_algoSatLos->z_signalBiasCorr))
    {
      break;
    }
    u_algoSignal = ssr_cvt_DyskSysCh2SignalType(p_dySatLos->sys, (uint8_t)p_dySatLos->code[u_ichan]);
    if (C_GNSS_SIG_MAX == u_algoSignal)
    {
      continue;
    }
    p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_signalType = u_algoSignal;
    p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_discontinuityIod = 0u;
    if (fabs(p_dySatLos->CodeCorr[u_ichan]) > 0.0)
    {
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_biasMask |= GNSS_SSR_SAT_BIAS_CODE_CORR;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_codeBias.f_qi = 0.0;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_codeBias.q_corr = (int32_t)(1000*p_dySatLos->CodeCorr[u_ichan]+0.5);
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_codeBias.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_codeBias.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
      ++u_algoChanNum;
    }
    if (fabs(p_dySatLos->PahseCorr[u_ichan]) > 0.0)
    {
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_biasMask |= GNSS_SSR_SAT_BIAS_PHASE_CORR;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_phaseBias.f_qi = 0.0;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_phaseBias.q_corr = (int32_t)(1000 * p_dySatLos->PahseCorr[u_ichan]+0.5);
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_phaseBias.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].z_phaseBias.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;

      p_algoSatLos->z_signalBiasCorr[u_algoChanNum].u_biasMask |= GNSS_SSR_SAT_BIAS_CODE_CORR; // for L1
    }
    
  }
  if (u_algoChanNum > 0)
  {
    u_validLos = 1;
  }
  p_algoSatLos->u_signalNum = u_algoChanNum;
  return u_validLos;
}

/**
 * @brief Convert SSR los from qxsi to asensing
 * @param[in] pz_qxsi_los - Qxsi SSR los data
 * @param[in] pz_ag_los - Asensing SSR los data
 * @return    Satellite Number
 */
uint8_t ssr_cvt_SsrLosBlk_Qxwz2Ag(const qxsi_station_los_t* pz_qxsi_los,
  gnss_ssrLosBlock_t* pz_ag_los)
{
  uint8_t u_satNum = 0;
  uint8_t isat = 0;
  uint8_t u_algoSys = C_GNSS_MAX;
  gnss_satSsrLos_t* p_algoSatLos = NULL;
  const qxsi_sat_los_t* p_qxSatLos = NULL;
  pz_ag_los->w_size = sizeof(gnss_ssrLosBlock_t);

  ssr_cvt_Time_Qxwz2Ag(&(pz_qxsi_los->obs_time), &(pz_ag_los->z_epochLosInfo.z_tor));
  ssr_cvt_Time_Qxwz2Ag(&(pz_qxsi_los->ztd_corr_time), &(pz_ag_los->z_epochLosInfo.z_ZTDtime));

  pz_ag_los->z_epochLosInfo.u_ZTDmask = ssr_cvt_AtmoMask_Qxwz2Ag(pz_qxsi_los->ztd_mask);
  pz_ag_los->z_epochLosInfo.u_errorModelMask = ssr_cvt_ErrorModelMask_Qxwz2Ag(pz_qxsi_los->site_error_model_mask);
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.f_qi = pz_qxsi_los->ztd.qi;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.d_dryCorr = pz_qxsi_los->ztd.dry_corr;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.d_wetCorr = pz_qxsi_los->ztd.wet_corr;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.u_postItg = ssr_cvt_IntegrityFlag_Qxwz2Ag(pz_qxsi_los->ztd.post_itg);
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.u_preItg = ssr_cvt_IntegrityFlag_Qxwz2Ag(pz_qxsi_los->ztd.pre_itg);
  pz_ag_los->z_epochLosInfo.u_satCount = u_satNum;
  for (isat = 0; isat < (pz_qxsi_los->sat_num); ++isat)
  {
    if (u_satNum >= GNSS_SSR_SAT_NUM_LIMIT)
    {
      break;
    }
    u_algoSys = ssr_cvt_cnstl_Qxwz2Ag(pz_qxsi_los->sat_los_corr[isat].id.sys, pz_qxsi_los->sat_los_corr[isat].id.prn);
    if (C_GNSS_MAX == u_algoSys)
    {
      continue;
    }
    p_qxSatLos = pz_qxsi_los->sat_los_corr + isat;
    p_algoSatLos = pz_ag_los->z_epochLosInfo.z_satLosCorr + u_satNum;
    p_algoSatLos->u_constellation = u_algoSys;
    p_algoSatLos->u_svid = (uint8_t)p_qxSatLos->id.prn;
    if (ssr_cvt_SatSsrLos_Qxwz2Ag(p_qxSatLos, p_algoSatLos))
    {
      ++u_satNum;
      ssr_cvt_Time_Qxwz2Ag(&(pz_qxsi_los->stec_time), &(p_algoSatLos->z_STECtime));
      ssr_cvt_Time_Qxwz2Ag(&(pz_qxsi_los->std_time), &(p_algoSatLos->z_STDtime));
    }
  }
  pz_ag_los->z_epochLosInfo.u_satCount = u_satNum;
  return u_satNum;
}
/**
 * @brief Convert SSR los from dayou to asensing
 * @param[in] pz_dysk_los - Dayou SSR los data
 * @param[in] pz_ag_los - Asensing SSR los data
 * @return    Satellite Number
 */
uint8_t ssr_cvt_SsrLosBlk_Dysk2Ag(const DySatLos_t* pz_dysk_los,
    gnss_ssrLosBlock_t* pz_ag_los)
{
  uint8_t u_satNum = 0;
  uint8_t isat = 0;
  uint8_t u_algoSys = C_GNSS_MAX;
  gnss_satSsrLos_t* p_algoSatLos = NULL;
  if (!pz_dysk_los || !pz_ag_los)
  {
    return 0;
  }

  pz_ag_los->w_size = sizeof(gnss_ssrLosBlock_t);

  ssr_cvt_Time_Dysk2Ag(&(pz_dysk_los->time), &(pz_ag_los->z_epochLosInfo.z_tor));
  pz_ag_los->z_epochLosInfo.u_ZTDmask = 0;
  pz_ag_los->z_epochLosInfo.u_errorModelMask = 0;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.f_qi = 0;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.d_dryCorr = 0;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.d_wetCorr = 0;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.u_postItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  pz_ag_los->z_epochLosInfo.z_ZTDcorr.u_preItg = GNSS_INTEGRITY_FLAG_MONITORED_OK;
  pz_ag_los->z_epochLosInfo.u_satCount = u_satNum;
  for (isat = 0; isat < DY_MAXOBS; ++isat)
  {
    if (u_satNum >= GNSS_SSR_SAT_NUM_LIMIT)
    {
      break;
    }
    u_algoSys = ssr_cvt_cnstl_Dysk2Ag(pz_dysk_los[isat].sys, pz_dysk_los[isat].prn);
    if (C_GNSS_MAX == u_algoSys|| pz_dysk_los[isat].prn<=0)
    {
      continue;
    }
    p_algoSatLos = pz_ag_los->z_epochLosInfo.z_satLosCorr + u_satNum;
    p_algoSatLos->u_constellation = u_algoSys;
    p_algoSatLos->u_svid = (uint8_t)pz_dysk_los[isat].prn;
    if (ssr_cvt_SatSsrLos_Dysk2Ag(&pz_dysk_los[isat], p_algoSatLos))
    {
      ++u_satNum;
    }
    if (p_algoSatLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR)
    {
      ssr_cvt_Time_Dysk2Ag(&(pz_dysk_los[isat].SsrTime), &(p_algoSatLos->z_STECtime));
    }
    if (p_algoSatLos->u_atmoMask & GNSS_SSR_ATMO_STD_CORR)
    {
      ssr_cvt_Time_Dysk2Ag(&(pz_dysk_los[isat].SsrTime), &(p_algoSatLos->z_STDtime));
    }
    
  }
  pz_ag_los->z_epochLosInfo.u_satCount = u_satNum;
  return u_satNum;
}
/**
 * @brief Convert constellation flag from Asensing to Qxsi
 * @param[in] e_ag_cnstl - Asensing's constellation flag
 * @return    QXWZ's constellation flag
 */
static qxsi_sat_sys_e ssr_cvt_cnstl_Ag2Qxwz(uint8_t e_ag_cnstl)
{
  qxsi_sat_sys_e e_qxsi_cnstl = QXSI_SAT_SYS_NONE;
  if (C_GNSS_GPS == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GPS;
  }
#if 0
  else if (C_GNSS_GLO == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GLO;
  }
#endif
  else if (C_GNSS_BDS3 == e_ag_cnstl || C_GNSS_BDS2 == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_BDS;
  }
  else if (C_GNSS_GAL == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GAL;
  }
#if 0
  else if (C_GNSS_QZS == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_QZS;
  }
#endif
  return e_qxsi_cnstl;
}
/**
 * @brief Convert Signal type from Asensing to QX sys and signal
 * @param[in] e_ag_signal_ch - Asensing's signal type
 * @return    QX signal type
 */
qxsi_channel_t ssr_cvt_ag_signalType2Qxwz(gnss_SignalType e_ag_signal_ch)
{
  qxsi_channel_t z_qxsi_signal_ch;
  z_qxsi_signal_ch.frq = 0;
  z_qxsi_signal_ch.tag = CHANNEL_IDX_HEAD;
  if (C_GNSS_SIG_GPS_L1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_GPS_L2C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_W;
  }
  else if (C_GNSS_SIG_GPS_L5Q == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (C_GNSS_SIG_GLO_G1 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_GLO_G2 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_BDS_B1I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (C_GNSS_SIG_BDS_B2I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (C_GNSS_SIG_BDS_B3I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 6;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (C_GNSS_SIG_BDS_B1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_X;
  }
  else if (C_GNSS_SIG_BDS_B2A == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_P;
  }
  else if (C_GNSS_SIG_BDS_B2B == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_P;
  }
  else if (C_GNSS_SIG_GAL_E1 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_GAL_E5A == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (C_GNSS_SIG_GAL_E5B == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (C_GNSS_SIG_QZS_L1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_QZS_L2C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (C_GNSS_SIG_QZS_L5Q == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  return z_qxsi_signal_ch;
}

/**
 * @brief Convert GNSS measurement block from asensing to qxsi
 * @param[in]  pz_site_coor -approximate coordinate of site
 * @param[in]  pz_ag_meas   - asensing GNSS measurement block
 * @param[out] pz_qxsi_station_info - qxsi station information
 * @return     0 represent failure, 1 represent success
 */
uint8_t cvt_GnssMeasBlk_Ag2Qxwz(const gnss_PositionFix_t* pz_posFix,
  const GnssMeasBlock_t* pz_ag_meas_block,
  qxsi_station_info_t* pz_qxsi_station_info)
{
  uint8_t u_status = 0;
  uint8_t u_qx_sat_num = 0;
  uint8_t u_qx_isat = 0;
  uint16_t w_ichan = 0;
  qxsi_sat_sys_e e_qxsi_cnstl = QXSI_SAT_SYS_NONE;
  const GnssMeas_t* pz_ag_meas = NULL;
  qxsi_sat_meas_data_t* pz_qxsi_sat_meas = NULL;
  int32_t q_qx_ichan = 0;
  qxsi_channel_t z_qxsi_signal_ch = { 0 };

  if (NULL == pz_posFix || NULL == pz_ag_meas_block || NULL == pz_qxsi_station_info)
  {
    return 0;
  }

  pz_qxsi_sat_meas = (pz_qxsi_station_info->obs.sat_data);
  // fillup the site coordinate to the qxsi struct
  pz_qxsi_station_info->pos.x = pz_posFix->d_xyz[0];
  pz_qxsi_station_info->pos.y = pz_posFix->d_xyz[1];
  pz_qxsi_station_info->pos.z = pz_posFix->d_xyz[2];
  // fillup the time to the qxsi struct
  pz_qxsi_station_info->obs.time.week_num = (int)pz_posFix->z_gpsTime.w_week;
  pz_qxsi_station_info->obs.time.sec_of_week = (double)(pz_posFix->z_gpsTime.q_towMsec * TIME_MSEC_INV);
  for (w_ichan = 0; w_ichan < (pz_ag_meas_block->w_measCount); ++w_ichan)
  {
    pz_ag_meas = (pz_ag_meas_block->z_meas + w_ichan);
    if ((pz_ag_meas->d_pseudoRange) <= 0.0)
    {
      continue;
    }
    e_qxsi_cnstl = ssr_cvt_cnstl_Ag2Qxwz(pz_ag_meas->u_constellation);
    if (QXSI_SAT_SYS_NONE == e_qxsi_cnstl)
    {
      continue;
    }
    for (u_qx_isat = 0; u_qx_isat < u_qx_sat_num; ++u_qx_isat)
    {
      if (pz_qxsi_sat_meas[u_qx_isat].id.sys == e_qxsi_cnstl &&
        pz_qxsi_sat_meas[u_qx_isat].id.prn == (int)pz_ag_meas->u_svid)
      {
        break;
      }
    }
    pz_qxsi_sat_meas[u_qx_isat].id.sys = e_qxsi_cnstl;
    pz_qxsi_sat_meas[u_qx_isat].id.prn = (int)pz_ag_meas->u_svid;
    if (u_qx_isat >= u_qx_sat_num)
    {
      ++u_qx_sat_num;
    }
    z_qxsi_signal_ch = ssr_cvt_ag_signalType2Qxwz(pz_ag_meas->u_signal);
    for (q_qx_ichan = 0; q_qx_ichan < (int32_t)pz_qxsi_sat_meas[u_qx_isat].num; ++q_qx_ichan)
    {
      if (z_qxsi_signal_ch.tag == pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel.tag
        && z_qxsi_signal_ch.frq == pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel.frq)
      {
        break;
      }
    }
    if (q_qx_ichan >= (int32_t)pz_qxsi_sat_meas[u_qx_isat].num)
    {
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel = z_qxsi_signal_ch;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.P = pz_ag_meas->d_pseudoRange;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.L = 0.0;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.D = 0.0;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.lli = 4;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.signal_strength = 9;
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.snr = 45;
      ++pz_qxsi_sat_meas[u_qx_isat].num;
    }
  }
  //fillup the satellite number to the qxsi struct
  pz_qxsi_station_info->obs.num = (int)u_qx_sat_num;
  if (u_qx_sat_num > 0)
  {
    u_status = 1;
  }
  return u_status;
}

/**
 * @brief Convert GNSS measurement block from asensing to qxsi
 * @param[in]  pz_site_coor -approximate coordinate of site
 * @param[in]  pz_SatSigMeasCollect   - asensing GNSS measurement block
 * @param[out] pz_qxsi_station_info - qxsi station information
 * @return     0 represent failure, 1 represent success
 */
uint8_t cvt_SatSigMeasCollect_Ag2Qxwz(const gnss_PositionFix_t* pz_posFix,
  const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect,
  qxsi_station_info_t* pz_qxsi_station_info)
{
  uint8_t u_status = 0;
  uint8_t u_qx_sat_num = 0;
  uint8_t u_qx_isat = 0;
  uint16_t w_ichan = 0;
  qxsi_sat_sys_e e_qxsi_cnstl = QXSI_SAT_SYS_NONE;
  qxsi_sat_meas_data_t* pz_qxsi_sat_meas = NULL;
  int32_t q_qx_ichan = 0;
  qxsi_channel_t z_qxsi_signal_ch = { 0 };

  const gnss_SatelliteMeas_t* pz_SatelliteMeas = NULL;
  const gnss_SignalMeas_t* pz_SignalMeas = NULL;
  uint8_t u_FreqIndex = 0;
  uint8_t u_satMeasIndex = 0;

  if (NULL == pz_posFix || NULL == pz_SatSigMeasCollect || NULL == pz_qxsi_station_info)
  {
    return 0;
  }

  pz_qxsi_sat_meas = (pz_qxsi_station_info->obs.sat_data);
  // fillup the site coordinate to the qxsi struct
  pz_qxsi_station_info->pos.x = pz_posFix->d_xyz[0];
  pz_qxsi_station_info->pos.y = pz_posFix->d_xyz[1];
  pz_qxsi_station_info->pos.z = pz_posFix->d_xyz[2];
  // fillup the time to the qxsi struct
  pz_qxsi_station_info->obs.time.week_num = (int)pz_posFix->z_gpsTime.w_week;
  pz_qxsi_station_info->obs.time.sec_of_week = (double)(pz_posFix->z_gpsTime.q_towMsec * TIME_MSEC_INV);

  for (w_ichan = 0; w_ichan < pz_SatSigMeasCollect->u_satMeasCount; ++w_ichan)
  {
    u_satMeasIndex = pz_SatSigMeasCollect->u_satMeasIdxTable[w_ichan];
    pz_SatelliteMeas = pz_SatSigMeasCollect->pz_satMeas[u_satMeasIndex];

    for (u_FreqIndex = 0; u_FreqIndex < MAX_GNSS_SIGNAL_FREQ; u_FreqIndex++)
    {
      pz_SignalMeas = pz_SatelliteMeas->pz_signalMeas[u_FreqIndex];
      if (NULL == pz_SignalMeas)
      {
        continue;
      }

      if (pz_SignalMeas->d_pseudoRange <= 0.0)
      {
        continue;
      }

      e_qxsi_cnstl = ssr_cvt_cnstl_Ag2Qxwz(pz_SignalMeas->u_constellation);
      if (QXSI_SAT_SYS_NONE == e_qxsi_cnstl)
      {
        continue;
      }
      for (u_qx_isat = 0; u_qx_isat < u_qx_sat_num; ++u_qx_isat)
      {
        if (pz_qxsi_sat_meas[u_qx_isat].id.sys == e_qxsi_cnstl &&
          pz_qxsi_sat_meas[u_qx_isat].id.prn == (int)pz_SignalMeas->u_svid)
        {
          break;
        }
      }
      pz_qxsi_sat_meas[u_qx_isat].id.sys = e_qxsi_cnstl;
      pz_qxsi_sat_meas[u_qx_isat].id.prn = (int)pz_SignalMeas->u_svid;
      if (u_qx_isat >= u_qx_sat_num)
      {
        ++u_qx_sat_num;
      }
      z_qxsi_signal_ch = ssr_cvt_ag_signalType2Qxwz(pz_SignalMeas->u_signal);
      for (q_qx_ichan = 0; q_qx_ichan < (int32_t)pz_qxsi_sat_meas[u_qx_isat].num; ++q_qx_ichan)
      {
        if (z_qxsi_signal_ch.tag == pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel.tag
          && z_qxsi_signal_ch.frq == pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel.frq)
        {
          break;
        }
      }
      if (q_qx_ichan >= (int32_t)pz_qxsi_sat_meas[u_qx_isat].num)
      {
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].channel = z_qxsi_signal_ch;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.P = pz_SignalMeas->d_pseudoRange;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.L = 0.0;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.D = 0.0;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.lli = 4;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.signal_strength = 9;
        pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.snr = 45;
        ++pz_qxsi_sat_meas[u_qx_isat].num;
      }
    }
  }
  //fillup the satellite number to the qxsi struct
  pz_qxsi_station_info->obs.num = (int)u_qx_sat_num;
  if (u_qx_sat_num > 0)
  {
    u_status = 1;
  }

  return u_status;
}