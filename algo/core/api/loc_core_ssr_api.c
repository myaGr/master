/**@file        loc_core_ssr_api.c
 * @brief       Location engine configuration
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/05  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "loc_core_ssr_api.h"
#include "loc_core_cfg.h"
#include "gnss_type.h"
#include "gnss_common.h"
#include "mw_log.h"

#include "mw_alloc.h"
#include "ppp_api.h"
#include "sm_api.h"

#include "qxsi_ssr2los_type_def.h"

extern loc_core_api_ctrl_t gz_loc_core_api_ctrl;

/**
* @brief Convert loc engine constellation type to internal constellation type
* @param[in]   u_LocConstellationType - Input constellation type
* @return      gnss_ConstellationType - Output internal constellation type
*/
static gnss_ConstellationType loc_ConvertConstellationLocToGnss(loc_api_gnssConstellationType u_LocConstellationType, uint8_t prn)
{
  gnss_ConstellationType u_type = LOC_API_GNSS_NONE;

  switch (u_LocConstellationType)
  {
  case LOC_API_GNSS_GPS:
    u_type = C_GNSS_GPS;
    break;
  case LOC_API_GNSS_GLO:
    u_type = C_GNSS_GLO;
    break;
  case LOC_API_GNSS_BDS:
    if (prn < 19)
    {
      u_type = C_GNSS_BDS2;
    }
    else
    {
      u_type = C_GNSS_BDS3;
    }
    break;
  case LOC_API_GNSS_GAL:
    u_type = C_GNSS_GAL;
    break;
  case LOC_API_GNSS_QZS:
    u_type = C_GNSS_QZS;
    break;
  default:
    u_type = LOC_API_GNSS_MAX;
    break;
  }
  return  u_type;
}
/**
 * @brief Convert SSR Los block from location core API to gnss internal
 * @param[in] pz_loc_ssrLosBlock - location core API's ssr los block
 * @param[in] pz_gnss_ssrLosBlock - gnss internal's ssr los block
 * @return    None
 */
static void ssr_Convert_LocApiSsrLosBlockToGnss(const loc_api_ssrLosBlock_t* pz_loc_ssrLosBlock, gnss_ssrLosBlock_t* pz_gnss_ssrLosBlock)
{
  if (!((0 == pz_loc_ssrLosBlock->u_version) && (VERSION_GNSS_SSR_LOS_BLOCK == 4)))
  {
    /* version of loc_api_ssrLosBlock_t must be 0 and
       version of gnss_ssrLosBlock_t must be 3,
       otherwise, need to upgrade structure */
    return;
  }

  pz_gnss_ssrLosBlock->u_version = VERSION_GNSS_SSR_LOS_BLOCK;
  pz_gnss_ssrLosBlock->w_size = pz_loc_ssrLosBlock->w_size;
  pz_gnss_ssrLosBlock->d_siteCoor[0] = pz_loc_ssrLosBlock->d_siteCoor[0];
  pz_gnss_ssrLosBlock->d_siteCoor[1] = pz_loc_ssrLosBlock->d_siteCoor[1];
  pz_gnss_ssrLosBlock->d_siteCoor[2] = pz_loc_ssrLosBlock->d_siteCoor[2];

  const loc_api_epochSsrLos_t* pz_srcSsrLos = &(pz_loc_ssrLosBlock->z_epochLosInfo);
  gnss_epochSsrLos_t* pz_dstSsrLos = &(pz_gnss_ssrLosBlock->z_epochLosInfo);

  tm_cvt_SetGpst(&pz_dstSsrLos->z_tor, pz_srcSsrLos->z_tor.w_week, pz_srcSsrLos->z_tor.tow);
  tm_cvt_SetGpst(&pz_dstSsrLos->z_ZTDtime, pz_srcSsrLos->z_ZTDtime.w_week, pz_srcSsrLos->z_ZTDtime.tow);
  pz_dstSsrLos->u_ZTDmask = pz_srcSsrLos->u_ZTDmask;
  pz_dstSsrLos->u_errorModelMask = pz_srcSsrLos->u_errorModelMask;
  pz_dstSsrLos->z_ZTDcorr.u_preItg = pz_srcSsrLos->z_ZTDcorr.u_preItg;
  pz_dstSsrLos->z_ZTDcorr.u_postItg = pz_srcSsrLos->z_ZTDcorr.u_postItg;
  pz_dstSsrLos->z_ZTDcorr.f_qi = pz_srcSsrLos->z_ZTDcorr.f_qi;
  pz_dstSsrLos->z_ZTDcorr.d_wetCorr = pz_srcSsrLos->z_ZTDcorr.d_wetCorr;
  pz_dstSsrLos->z_ZTDcorr.d_dryCorr = pz_srcSsrLos->z_ZTDcorr.d_dryCorr;
  pz_dstSsrLos->u_satCount = pz_srcSsrLos->u_satCount;

  for (uint8_t i = 0; i < pz_dstSsrLos->u_satCount; i++)
  {
    const loc_api_satSsrLos_t* pz_srcSatSsrLoc = &pz_srcSsrLos->z_satLosCorr[i];
    gnss_satSsrLos_t* pz_dstSatSsrLoc = &pz_dstSsrLos->z_satLosCorr[i];

    pz_dstSatSsrLoc->u_constellation = loc_ConvertConstellationLocToGnss(pz_srcSatSsrLoc->u_constellation, pz_srcSatSsrLoc->u_svid);
    pz_dstSatSsrLoc->u_svid = pz_srcSatSsrLoc->u_svid;
    pz_dstSatSsrLoc->u_orbClkMask = pz_srcSatSsrLoc->u_orbClkMask;
    pz_dstSatSsrLoc->u_atmoMask = pz_srcSatSsrLoc->u_atmoMask;
    pz_dstSatSsrLoc->u_windUpMask = pz_srcSatSsrLoc->u_windUpMask;
    pz_dstSatSsrLoc->u_signalNum = pz_srcSatSsrLoc->u_signalNum;
    pz_dstSatSsrLoc->q_iode = pz_srcSatSsrLoc->q_iode;

    const loc_api_SatPosVelClk_t* p_srcSatPosVelClk = &(pz_srcSatSsrLoc->z_satPosVelClkBrdc);
    gnss_SatPosVelClk_t* p_dstSatPosVelClk = &(pz_dstSatSsrLoc->z_satPosVelClkBrdc);
    tm_cvt_SetGpst(&p_dstSatPosVelClk->z_gpsTime, p_srcSatPosVelClk->z_gpsTime.w_week, p_srcSatPosVelClk->z_gpsTime.tow);
    p_dstSatPosVelClk->u_valid = p_srcSatPosVelClk->u_valid;
    p_dstSatPosVelClk->u_constellation = loc_ConvertConstellationLocToGnss(p_srcSatPosVelClk->u_constellation, p_srcSatPosVelClk->u_svid);
    p_dstSatPosVelClk->u_svid = p_srcSatPosVelClk->u_svid;
    p_dstSatPosVelClk->f_elevation = p_srcSatPosVelClk->f_elevation;
    p_dstSatPosVelClk->f_azimuth = p_srcSatPosVelClk->f_azimuth;
    p_dstSatPosVelClk->f_dt = p_srcSatPosVelClk->f_dt;
    p_dstSatPosVelClk->q_iode = p_srcSatPosVelClk->q_iode;
    for (uint8_t k = 0; k < 4; k++)
    {
      p_dstSatPosVelClk->d_satPosClk[k] = p_srcSatPosVelClk->d_satPosClk[k];
      p_dstSatPosVelClk->d_satVelClk[k] = p_srcSatPosVelClk->d_satVelClk[k];
    }
    pz_dstSatSsrLoc->u_clockContinuityIod = pz_srcSatSsrLoc->u_clockContinuityIod;
    pz_dstSatSsrLoc->z_orbClk.u_preItg = pz_srcSatSsrLoc->z_orbClk.u_preItg;
    pz_dstSatSsrLoc->z_orbClk.u_postItg = pz_srcSatSsrLoc->z_orbClk.u_postItg;
    pz_dstSatSsrLoc->z_orbClk.f_qi = pz_srcSatSsrLoc->z_orbClk.f_qi;
    pz_dstSatSsrLoc->z_orbClk.q_corr = pz_srcSatSsrLoc->z_orbClk.q_corr;

    pz_dstSatSsrLoc->z_orbclkPrec.u_preItg = pz_srcSatSsrLoc->z_orbclkPrec.u_preItg;
    pz_dstSatSsrLoc->z_orbclkPrec.u_postItg = pz_srcSatSsrLoc->z_orbclkPrec.u_postItg;
    pz_dstSatSsrLoc->z_orbclkPrec.f_qi = pz_srcSatSsrLoc->z_orbclkPrec.f_qi;
    pz_dstSatSsrLoc->z_orbclkPrec.q_clock_bias = pz_srcSatSsrLoc->z_orbclkPrec.q_clock_bias;
    for (uint8_t k = 0; k < 3; k++)
    {
      pz_dstSatSsrLoc->z_orbclkPrec.q_delta_pos[k] = pz_srcSatSsrLoc->z_orbclkPrec.q_delta_pos[k];
    }
    tm_cvt_SetGpst(&pz_dstSatSsrLoc->z_orbClkTime, pz_srcSatSsrLoc->z_orbClkTime.w_week, pz_srcSatSsrLoc->z_orbClkTime.tow);

    tm_cvt_SetGpst(&pz_dstSatSsrLoc->z_STECtime, pz_srcSatSsrLoc->z_STECtime.w_week, pz_srcSatSsrLoc->z_STECtime.tow);
    tm_cvt_SetGpst(&pz_dstSatSsrLoc->z_STDtime, pz_srcSatSsrLoc->z_STDtime.w_week, pz_srcSatSsrLoc->z_STDtime.tow);

    pz_dstSatSsrLoc->z_stec.u_preItg = pz_srcSatSsrLoc->z_stec.u_preItg;
    pz_dstSatSsrLoc->z_stec.u_postItg = pz_srcSatSsrLoc->z_stec.u_postItg;
    pz_dstSatSsrLoc->z_stec.f_qi = pz_srcSatSsrLoc->z_stec.f_qi;
    pz_dstSatSsrLoc->z_stec.q_corr = pz_srcSatSsrLoc->z_stec.q_corr;

    pz_dstSatSsrLoc->z_std.u_preItg = pz_srcSatSsrLoc->z_std.u_preItg;
    pz_dstSatSsrLoc->z_std.u_postItg = pz_srcSatSsrLoc->z_std.u_postItg;
    pz_dstSatSsrLoc->z_std.f_qi = pz_srcSatSsrLoc->z_std.f_qi;
    pz_dstSatSsrLoc->z_std.q_corr = pz_srcSatSsrLoc->z_std.q_corr;

    pz_dstSatSsrLoc->d_windUp = pz_srcSatSsrLoc->d_windUp;
    pz_dstSatSsrLoc->f_yaw = pz_srcSatSsrLoc->f_yaw;
    pz_dstSatSsrLoc->f_yawRate = pz_srcSatSsrLoc->f_yawRate;

    for (uint8_t j = 0; j < GNSS_SSR_SAT_CHL_NUM_LIMIT; j++)
    {
      const loc_api_SignalCorr_t* p_srcSignalCorr = &(pz_srcSatSsrLoc->z_signalBiasCorr[j]);
      gnss_SignalCorr_t* p_dstSignalCorr = &(pz_dstSatSsrLoc->z_signalBiasCorr[j]);

      p_dstSignalCorr->u_signalType = p_srcSignalCorr->u_signalType;
      p_dstSignalCorr->u_biasMask = p_srcSignalCorr->u_biasMask;
      p_dstSignalCorr->f_pcv = p_srcSignalCorr->f_pcv;
      p_dstSignalCorr->u_discontinuityIod = p_srcSignalCorr->u_discontinuityIod;

      p_dstSignalCorr->z_codeBias.u_preItg = p_srcSignalCorr->z_codeBias.u_preItg;
      p_dstSignalCorr->z_codeBias.u_postItg = p_srcSignalCorr->z_codeBias.u_postItg;
      p_dstSignalCorr->z_codeBias.f_qi = p_srcSignalCorr->z_codeBias.f_qi;
      p_dstSignalCorr->z_codeBias.q_corr = p_srcSignalCorr->z_codeBias.q_corr;

      p_dstSignalCorr->z_phaseBias.u_preItg = p_srcSignalCorr->z_phaseBias.u_preItg;
      p_dstSignalCorr->z_phaseBias.u_postItg = p_srcSignalCorr->z_phaseBias.u_postItg;
      p_dstSignalCorr->z_phaseBias.f_qi = p_srcSignalCorr->z_phaseBias.f_qi;
      p_dstSignalCorr->z_phaseBias.q_corr = p_srcSignalCorr->z_phaseBias.q_corr;
    }
  }

  return;
}

/**
 * @brief Convert constellation flag from Asensing to Qxsi
 * @param[in] u_loc_constellation - Asensing's constellation flag
 * @return    QXWZ's constellation flag
 */
static qxsi_sat_sys_e ssr_Convert_LocApiConstellationToQxsiSatSys(loc_api_gnssConstellationType u_loc_constellation)
{
  qxsi_sat_sys_e e_qxsi_cnstl = QXSI_SAT_SYS_NONE;
  if (LOC_API_GNSS_GPS == u_loc_constellation)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GPS;
  }
#if 0
  else if (LOC_API_GNSS_GLN == e_ag_cnstl)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GLO;
  }
#endif
  else if (LOC_API_GNSS_BDS == u_loc_constellation)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_BDS;
  }
  else if (LOC_API_GNSS_GAL == u_loc_constellation)
  {
    e_qxsi_cnstl = QXSI_SAT_SYS_GAL;
  }
#if 0
  else if (LOC_API_GNSS_QZS == e_ag_cnstl)
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
static qxsi_channel_t ssr_Convert_LocApiSignalTypeToQxsiChannel(loc_api_SignalType e_ag_signal_ch)
{
  qxsi_channel_t z_qxsi_signal_ch;
  z_qxsi_signal_ch.frq = 0;
  z_qxsi_signal_ch.tag = CHANNEL_IDX_HEAD;
  if (LOC_API_GNSS_SIG_GPS_L1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_GPS_L2C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_L;
  }
  else if (LOC_API_GNSS_SIG_GPS_L5Q == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (LOC_API_GNSS_SIG_GLO_G1 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_GLO_G2 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_BDS_B1I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (LOC_API_GNSS_SIG_BDS_B2I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (LOC_API_GNSS_SIG_BDS_B3I == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 6;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_I;
  }
  else if (LOC_API_GNSS_SIG_BDS_B1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_X;
  }
  else if (LOC_API_GNSS_SIG_BDS_B2A == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_P;
  }
  else if (LOC_API_GNSS_SIG_BDS_B2B == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_P;
  }
  else if (LOC_API_GNSS_SIG_GAL_E1 == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_GAL_E5A == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (LOC_API_GNSS_SIG_GAL_E5B == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 7;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  else if (LOC_API_GNSS_SIG_QZS_L1C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 1;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_QZS_L2C == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 2;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_C;
  }
  else if (LOC_API_GNSS_SIG_QZS_L5Q == e_ag_signal_ch)
  {
    z_qxsi_signal_ch.frq = 5;
    z_qxsi_signal_ch.tag = CHANNEL_IDX_Q;
  }
  return z_qxsi_signal_ch;
}

/**
 * @brief Convert Loc Api reported measurement to Qianxun station input info
 * @param[in] pz_loc_GnssMeasBlk - location core API's reported measurement
 * @param[in] pz_qxsi_station_info - Qianxun station input info
 * @return    TRUE - Success
 *            FALSE - Fail
 */
uint8_t ssr_Convert_LocApiGnssMeasBlkToQxsiStationInfo(const loc_api_GnssMeasBlock_t* pz_loc_GnssMeasBlk,
                                                       qxsi_station_info_t* pz_qxsi_station_info)
{
  uint8_t u_status = 0;
  uint8_t u_qx_sat_num = 0;
  uint8_t u_qx_isat = 0;
  uint16_t w_ichan = 0;
  qxsi_sat_meas_data_t* pz_qxsi_sat_meas = NULL;
  int32_t q_qx_ichan = 0;
  qxsi_channel_t z_qxsi_signal_ch = { 0 };

  if (NULL == pz_loc_GnssMeasBlk || NULL == pz_qxsi_station_info)
  {
    return 0;
  }
  pz_qxsi_sat_meas = (pz_qxsi_station_info->obs.sat_data);
  // fillup the site coordinate to the qxsi struct
  double d_xyz[3] = { 0 };
  gnss_Lla2Ecef(pz_loc_GnssMeasBlk->d_lla, d_xyz);
  pz_qxsi_station_info->pos.x = d_xyz[0];
  pz_qxsi_station_info->pos.y = d_xyz[1];
  pz_qxsi_station_info->pos.z = d_xyz[2];
  // fillup the time to the qxsi struct
  pz_qxsi_station_info->obs.time.week_num = (int)pz_loc_GnssMeasBlk->w_week;
  pz_qxsi_station_info->obs.time.sec_of_week = (double)(pz_loc_GnssMeasBlk->q_towMsec * TIME_MSEC_INV);

  for (w_ichan = 0; w_ichan < (pz_loc_GnssMeasBlk->w_measCount); ++w_ichan)
  {
    const loc_api_GnssMeas_t* pz_loc_GnssMeas = &pz_loc_GnssMeasBlk->z_GnssMeas[w_ichan];
    if ((pz_loc_GnssMeas->d_pseudoRange) <= 0.0)
    {
      continue;
    }

    qxsi_sat_sys_e e_qxsi_cnstl = ssr_Convert_LocApiConstellationToQxsiSatSys(pz_loc_GnssMeas->u_constellation);
    if (QXSI_SAT_SYS_NONE == e_qxsi_cnstl)
    {
      continue;
    }

    for (u_qx_isat = 0; u_qx_isat < u_qx_sat_num; ++u_qx_isat)
    {
      if (pz_qxsi_sat_meas[u_qx_isat].id.sys == e_qxsi_cnstl &&
          pz_qxsi_sat_meas[u_qx_isat].id.prn == (int)pz_loc_GnssMeas->u_svid)
      {
        break;
      }
    }

    pz_qxsi_sat_meas[u_qx_isat].id.sys = e_qxsi_cnstl;
    pz_qxsi_sat_meas[u_qx_isat].id.prn = (int)pz_loc_GnssMeas->u_svid;
    if (u_qx_isat >= u_qx_sat_num)
    {
      ++u_qx_sat_num;
    }
    z_qxsi_signal_ch = ssr_Convert_LocApiSignalTypeToQxsiChannel(pz_loc_GnssMeas->u_signal);
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
      pz_qxsi_sat_meas[u_qx_isat].chl_data[q_qx_ichan].data.P = pz_loc_GnssMeas->d_pseudoRange;
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
 * @brief Convert atmo mask from Qianxun to location core API
 * @param[in] u_qx_atmo_mask - Qianxun's atmo mask
 * @return    Location core's atmo mask
 */
static loc_api_ssrAtmoMask ssr_Convert_QxsiSsrAtmoMaskToLocApi(qxsi_ssr_atmo_mask u_qx_atmo_mask)
{
  gnss_ssrAtmoMask u_loc_atmo_mask = 0;
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_STEC_CORR)
  {
    u_loc_atmo_mask |= LOC_API_SSR_ATMO_STEC_CORR;
  }
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_STD_CORR)
  {
    u_loc_atmo_mask |= LOC_API_SSR_ATMO_STD_CORR;
  }
  if (u_qx_atmo_mask & QXSI_SSR_ATMO_ZTD_CORR)
  {
    u_loc_atmo_mask |= LOC_API_SSR_ATMO_ZTD_CORR;
  }
  return u_loc_atmo_mask;
}

/**
 * @brief Convert error model mask from Qianxun to location core API
 * @param[in] u_qx_error_mask - Qianxun's error model mask
 * @return    Location core'serror model mask
 */
static loc_api_ssrErrorModelMask ssr_Convert_QxsiSsrErrorModelMaskToLocApi(
  qxsi_ssr_site_error_model_mask u_qx_error_mask)
{
  gnss_ssrErrorModelMask u_loc_error_mask = 0;
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_SOLID_TIDE_CORR)
  {
    u_loc_error_mask |= LOC_API_SSR_ERROR_MODEL_SOLID_TIDE_CORR;
  }
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_OCEAN_TIDE_CORR)
  {
    u_loc_error_mask |= LOC_API_SSR_ERROR_MODEL_OCEAN_TID_CORR;
  }
  if (u_qx_error_mask & QXSI_SSR_ERROR_MODEL_SITE_POLE_TIDE_CORR)
  {
    u_loc_error_mask |= LOC_API_SSR_ERROR_MODEL_POLE_TIDE_CORR;
  }
  return u_loc_error_mask;
}

/**
 * @brief Convert integrity flag from Qxsi to location core API
 * @param[in] u_qx_integrity_flag - Qianxun's integrity flag
 * @return    Location core's integrity flag
 */
static loc_api_integrityFlag  ssr_Convert_QxsiIntegrityFlagToLocApi(qxsi_integrity_flag u_qx_integrity_flag)
{
  loc_api_integrityFlag u_loc_integrity_flag = LOC_API_INTEGRITY_FLAG_MONITORED_OK;
  if (QXSI_INTEGRITY_FLAG_MONITORED_OK == u_qx_integrity_flag)
  {
    u_loc_integrity_flag = LOC_API_INTEGRITY_FLAG_MONITORED_OK;
  }
  else if (QXSI_INTEGRITY_FLAG_MONITORED_FAIL == u_qx_integrity_flag)
  {
    u_loc_integrity_flag = LOC_API_INTEGRITY_FLAG_MONITORED_FAIL;
  }
  else if (QXSI_INTEGRITY_FLAG_NOT_MONITORED == u_qx_integrity_flag)
  {
    u_loc_integrity_flag = LOC_API_INTEGRITY_FLAG_NOT_MONITORED;
  }
  return u_loc_integrity_flag;
}

/**
 * @brief Convert constellation flag from Qxsi to location core API
 * @param[in] qx_cnstl - Qianxun's constellation flag
 * @return    location core's constellation flag
 */
static loc_api_gnssConstellationType ssr_Convert_QxsiSatSysToLocApi(qxsi_sat_sys_e qx_cnstl)
{
  loc_api_gnssConstellationType u_loc_constellation = LOC_API_GNSS_MAX;
  if (QXSI_SAT_SYS_GPS == qx_cnstl)
  {
    u_loc_constellation = LOC_API_GNSS_GPS;
  }
  else if (QXSI_SAT_SYS_BDS == qx_cnstl)
  {
    u_loc_constellation = LOC_API_GNSS_BDS;
  }
  else if (QXSI_SAT_SYS_GAL == qx_cnstl)
  {
    u_loc_constellation = LOC_API_GNSS_GAL;
  }
  return u_loc_constellation;
}

/**
 * @brief Convert orbit clock mask from Qxsi to location core API
 * @param[in] u_qx_orb_clk_mask - Qianxun's orbit clock mask
 * @return    location core's orbit clock mask
 */
static loc_api_ssrSatOrbClkMask ssr_Convert_QxsiOrbClkMaskToLocApi(qxsi_ssr_sat_orb_clk_mask u_qx_orb_clk_mask)
{
  loc_api_ssrSatOrbClkMask u_loc_orb_clk_mask = 0;
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_EPH)
  {
    u_loc_orb_clk_mask |= LOC_API_SSR_SAT_ORB_CLK_EPH;
  }
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_ORBIT_CORR)
  {
    u_loc_orb_clk_mask |= LOC_API_SSR_SAT_ORB_CLK_ORBIT_CORR;
  }
  if (u_qx_orb_clk_mask & QXSI_SSR_SAT_ORB_CLK_CLOCK_CORR)
  {
    u_loc_orb_clk_mask |= LOC_API_SSR_SAT_ORB_CLK_CLOCK_CORR;
  }
  return u_loc_orb_clk_mask;
}

/**
 * @brief Convert correction bias from Qxsi to location core API
 * @param[in] pz_qx_ssr_corr - Qianxun's correction bias
 * @param[out] pz_loc_SsrCorrBias - location core's correction bias
 * @return    None
 */
static void ssr_Convert_QxsiSsrCorrBiasToLocApi(const qxsi_corr_data_t* pz_qx_ssr_corr, loc_api_SsrCorrBias_t* pz_loc_SsrCorrBias)
{
  if ((NULL == pz_qx_ssr_corr) || (NULL == pz_loc_SsrCorrBias))
  {
    return;
  }

  pz_loc_SsrCorrBias->u_postItg = ssr_Convert_QxsiIntegrityFlagToLocApi(pz_qx_ssr_corr->post_itg);
  pz_loc_SsrCorrBias->u_preItg = ssr_Convert_QxsiIntegrityFlagToLocApi(pz_qx_ssr_corr->pre_itg);
  pz_loc_SsrCorrBias->f_qi = pz_qx_ssr_corr->qi;
  pz_loc_SsrCorrBias->q_corr = (int32_t)(pz_qx_ssr_corr->corr * 1000.0 + 0.5);

  return;
}

/**
 * @brief Get Signal type from Qianxun sys and signal
 * @param[in] sat_sys - Qianxun's correction bias
 * @param[in] qx_signal - Qianxun's correction bias
 * @return    gnss signal type
 */
static loc_api_SignalType ssr_Convert_QxsiSysChannelToLocApiSignalType(qxsi_sat_sys_e qxsi_sat_sys,
                                                          qxsi_channel_t pz_qxsi_signal_ch)
{
  loc_api_SignalType u_signalType = LOC_API_GNSS_SIG_MAX;
  if (QXSI_SAT_SYS_GPS == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GPS_L1C;
      }
    }
    else if (2 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GPS_L2C;
      }
      if (CHANNEL_IDX_L == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GPS_L2C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_Q == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GPS_L5Q;
      }
    }
  }
#if 0
  else if (QXSI_SAT_SYS_GLO == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GLO_G1;
      }
    }
    else if (2 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_GLO_G2;
      }
    }
  }
#endif
  else if (QXSI_SAT_SYS_BDS == qxsi_sat_sys)
  {
    if (2 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_BDS_B1I;
      }
    }
    else if (7 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_BDS_B2I;
      }
    }
    else if (6 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_I == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_BDS_B3I;
      }
    }
    else if (1 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_P == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_BDS_B1C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_P == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_BDS_B2A;
      }
    }
  }
  else if (QXSI_SAT_SYS_GAL == qxsi_sat_sys)//to be improved
  {
    if (1 == (pz_qxsi_signal_ch.frq))
    {
      u_signalType = LOC_API_GNSS_SIG_GAL_E1;
    }
    else if (7 == (pz_qxsi_signal_ch.frq))
    {
      u_signalType = LOC_API_GNSS_SIG_GAL_E5B;
    }
    else if (5 == (pz_qxsi_signal_ch.frq))
    {
      u_signalType = LOC_API_GNSS_SIG_GAL_E5A;
    }
  }
#if 0
  else if (QXSI_SAT_SYS_QZS == qxsi_sat_sys)
  {
    if (1 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_QZS_L1C;
      }
    }
    else if (2 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_C == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_QZS_L2C;
      }
    }
    else if (5 == (pz_qxsi_signal_ch.frq))
    {
      if (CHANNEL_IDX_Q == (pz_qxsi_signal_ch.tag))
      {
        u_signalType = LOC_API_GNSS_SIG_QZS_L5Q;
      }
    }
  }
#endif
  return u_signalType;
}

/**
 * @brief Convert Satellites bias mask from Qxsi to Asensing
 * @param[in] u_qx_error_mask - Qianxun's Satellites bias mask
 * @return    Asensing's Satellites bias mask
 */
static loc_api_ssrSatBiasMask ssr_Convert_QxwzSatBiasMaskToLocApiSatBaisMask(qxsi_ssr_sat_bias_mask u_qx_sat_bias_mask)
{
  gnss_ssrSatBiasMask u_loc_sat_bias_mask = 0;
  if (u_qx_sat_bias_mask & QXSI_SSR_SAT_BIAS_CODE_CORR)
  {                                                     
    u_loc_sat_bias_mask |= LOC_API_SSR_SAT_BIAS_CODE_CORR;
  }
  if (u_qx_sat_bias_mask & QXSI_SSR_SAT_BIAS_PHASE_CORR)
  {
    u_loc_sat_bias_mask |= LOC_API_SSR_SAT_BIAS_PHASE_CORR;
  }
  return u_loc_sat_bias_mask;
}

/**
 * @brief Convert Qianxun sat los structure to Location core Ssr los structure
 * @param[in] p_qx_sat_los : Qianxun sat los structure 
 * @param[in] p_loc_sat_los : Location core Ssr los structure
 * @return    TRUE  : Success
 *            FALSE : Fail
 */
static uint8_t ssr_Convert_QxsiSatLosToLocApiSatSsrLos(const qxsi_sat_los_t* p_qx_sat_los, loc_api_satSsrLos_t* p_loc_sat_los)
{
  uint8_t u_SignalBiasCorrCount = 0;

  p_loc_sat_los->u_orbClkMask = ssr_Convert_QxsiOrbClkMaskToLocApi(p_qx_sat_los->orb_clk_mask);
  p_loc_sat_los->u_atmoMask = ssr_Convert_QxsiSsrAtmoMaskToLocApi(p_qx_sat_los->atmo_mask);
  p_loc_sat_los->u_windUpMask = ssr_Convert_QxsiSsrErrorModelMaskToLocApi(p_qx_sat_los->sat_error_model_mask);
  p_loc_sat_los->q_iode = (int32_t)p_qx_sat_los->brdc_info.iode;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satPosClk[0] = p_qx_sat_los->brdc_info.pos.x;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satPosClk[1] = p_qx_sat_los->brdc_info.pos.y;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satPosClk[2] = p_qx_sat_los->brdc_info.pos.z;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satPosClk[3] = p_qx_sat_los->brdc_info.clock;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satVelClk[0] = p_qx_sat_los->brdc_info.vel.x;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satVelClk[1] = p_qx_sat_los->brdc_info.vel.y;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satVelClk[2] = p_qx_sat_los->brdc_info.vel.z;
  p_loc_sat_los->z_satPosVelClkBrdc.d_satVelClk[3] = p_qx_sat_los->brdc_info.clock_drift;

  ssr_Convert_QxsiSsrCorrBiasToLocApi(&p_qx_sat_los->orb_clk, &p_loc_sat_los->z_orbClk);
  p_loc_sat_los->z_orbClkTime.w_week = p_qx_sat_los->orb_clk_time.week_num;
  p_loc_sat_los->z_orbClkTime.tow = p_qx_sat_los->orb_clk_time.sec_of_week;
  ssr_Convert_QxsiSsrCorrBiasToLocApi(&p_qx_sat_los->stec, &p_loc_sat_los->z_stec);
  ssr_Convert_QxsiSsrCorrBiasToLocApi(&p_qx_sat_los->std, &p_loc_sat_los->z_std);

  p_loc_sat_los->d_windUp = p_qx_sat_los->windup;
  p_loc_sat_los->f_yaw = p_qx_sat_los->yaw;
  p_loc_sat_los->f_yawRate = p_qx_sat_los->yaw_rate;
  for (uint8_t u_ichan = 0; u_ichan < (p_qx_sat_los->signal_num); ++u_ichan)
  {
    if (u_SignalBiasCorrCount >= sizeof(p_loc_sat_los->z_signalBiasCorr) / sizeof(p_loc_sat_los->z_signalBiasCorr[0]))
    {
      break;
    }

    const qxsi_signal_corr_t* p_qx_signal_corr = &(p_qx_sat_los->signal_bias[u_ichan]);
    loc_api_SignalType u_Signaltype = ssr_Convert_QxsiSysChannelToLocApiSignalType(p_qx_sat_los->id.sys, p_qx_signal_corr->signal_type);
    if (C_GNSS_SIG_MAX == u_Signaltype)
    {
      continue;
    }

    loc_api_SignalCorr_t* p_dstSignalCorr = &(p_loc_sat_los->z_signalBiasCorr[u_SignalBiasCorrCount]);
    u_SignalBiasCorrCount++;

    p_dstSignalCorr->u_signalType = u_Signaltype;
    p_dstSignalCorr->u_discontinuityIod = p_qx_signal_corr->phase_bias_continue_flag;
    p_dstSignalCorr->u_biasMask = ssr_Convert_QxwzSatBiasMaskToLocApiSatBaisMask(p_qx_signal_corr->bias_mask);

    ssr_Convert_QxsiSsrCorrBiasToLocApi(&p_qx_signal_corr->P, &p_dstSignalCorr->z_codeBias);
    ssr_Convert_QxsiSsrCorrBiasToLocApi(&p_qx_signal_corr->L, &p_dstSignalCorr->z_phaseBias);
  }

  p_loc_sat_los->u_signalNum = u_SignalBiasCorrCount;

  if (0 == u_SignalBiasCorrCount)
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief Convert SSR los block from Qianxun to location core API
 * @param[in] pz_qxsi_los - Qianxun's station los
 * @param[in] pz_SsrLosBlk - Location core's SSR los
 * @return    TRUE - Success
 *            FALSE - Fail
 */
uint8_t ssr_Convert_QxsiStationLosToLocApiSsrLosBlk(const qxsi_station_los_t* pz_qxsi_los,
                                                   loc_api_ssrLosBlock_t* pz_SsrLosBlk)
{
  uint8_t u_satNum = 0;
  loc_api_gnssConstellationType u_constellation = LOC_API_GNSS_MAX;

  pz_SsrLosBlk->w_size = sizeof(loc_api_ssrLosBlock_t);

  loc_api_epochSsrLos_t* pz_ApiEpochSsrLos = &(pz_SsrLosBlk->z_epochLosInfo);

  pz_ApiEpochSsrLos->z_tor.w_week        = pz_qxsi_los->obs_time.week_num;
  pz_ApiEpochSsrLos->z_tor.tow           = pz_qxsi_los->obs_time.sec_of_week;
  pz_ApiEpochSsrLos->z_ZTDtime.w_week    = pz_qxsi_los->ztd_corr_time.week_num;
  pz_ApiEpochSsrLos->z_ZTDtime.tow       = pz_qxsi_los->ztd_corr_time.sec_of_week;     
  pz_ApiEpochSsrLos->u_ZTDmask           = ssr_Convert_QxsiSsrAtmoMaskToLocApi(pz_qxsi_los->ztd_mask);
  pz_ApiEpochSsrLos->u_errorModelMask    = ssr_Convert_QxsiSsrErrorModelMaskToLocApi(pz_qxsi_los->site_error_model_mask);
  pz_ApiEpochSsrLos->z_ZTDcorr.f_qi      = pz_qxsi_los->ztd.qi;
  pz_ApiEpochSsrLos->z_ZTDcorr.d_dryCorr = pz_qxsi_los->ztd.dry_corr;
  pz_ApiEpochSsrLos->z_ZTDcorr.d_wetCorr = pz_qxsi_los->ztd.wet_corr;
  pz_ApiEpochSsrLos->z_ZTDcorr.u_postItg = ssr_Convert_QxsiIntegrityFlagToLocApi(pz_qxsi_los->ztd.post_itg);
  pz_ApiEpochSsrLos->z_ZTDcorr.u_preItg  = ssr_Convert_QxsiIntegrityFlagToLocApi(pz_qxsi_los->ztd.pre_itg);
  pz_ApiEpochSsrLos->u_satCount          = pz_qxsi_los->sat_num;
  for (uint8_t i = 0; i < pz_ApiEpochSsrLos->u_satCount; ++i)
  {
    if (u_satNum >= GNSS_SSR_SAT_NUM_LIMIT)
    {
      break;
    }

    u_constellation = ssr_Convert_QxsiSatSysToLocApi(pz_qxsi_los->sat_los_corr[i].id.sys);
    if (LOC_API_GNSS_MAX == u_constellation)
    {
      continue;
    }
    const qxsi_sat_los_t* p_qxSatLos = &pz_qxsi_los->sat_los_corr[i];
    loc_api_satSsrLos_t* pz_SatSsrLos = &pz_ApiEpochSsrLos->z_satLosCorr[u_satNum];

    u_satNum++;
    
    pz_SatSsrLos->u_constellation = u_constellation;
    pz_SatSsrLos->u_svid = (uint8_t)p_qxSatLos->id.prn;

    ssr_Convert_QxsiSatLosToLocApiSatSsrLos(p_qxSatLos, pz_SatSsrLos);


    pz_SatSsrLos->z_STECtime.w_week = pz_qxsi_los->stec_time.week_num;
    pz_SatSsrLos->z_STECtime.tow    = pz_qxsi_los->stec_time.sec_of_week;
    pz_SatSsrLos->z_STDtime.w_week  = pz_qxsi_los->std_time.week_num;
    pz_SatSsrLos->z_STDtime.tow     = pz_qxsi_los->std_time.sec_of_week;
  }

  return 0;
}

/**
  * @brief     Inject Observation LOS of SSR data
  * @param[in] pz_LocSsrLosBlk - pointer to gnss_ssrLosBlock_t
  * @return    None
  * @note      gnss_ssrLosBlock_t is defined by Location engine, it needs to
  *     convert SSR service's data to this structure before call the function
  */
void loc_api_InjectSsrLosBlock(const loc_api_ssrLosBlock_t* pz_LocSsrLosBlk)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit||NULL==pz_LocSsrLosBlk)
  {
    return;
  }
  gnss_ssrLosBlock_t* pz_ssrLosBlock = OS_MALLOC(sizeof(gnss_ssrLosBlock_t));
  if (NULL == pz_ssrLosBlock)
  {
    return;
  }

  ssr_Convert_LocApiSsrLosBlockToGnss(pz_LocSsrLosBlk, pz_ssrLosBlock);

  ppp_api_SsrLocBlock_Put(pz_ssrLosBlock);

  OS_FREE(pz_ssrLosBlock);
  return;
}

#if 0  // Comment the code but don't remove
static void loc_api_InjectSsrStream_by_AGssr(uint8_t* data, uint32_t len)
{
  static SSRDecoder_t ssr_decoder;
  int32_t ret = 0;
  ipc_t ipc = { 0 };
  do
  {
    ret = parse_ssr_stream(&ssr_decoder, (char*)data, len);
    if (LOC_REPORT_ID_SSR_LOS_BLK == ret)
    {
      ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_SSR_AG_LOS, (uint8_t*)ssr_decoder.payload, ssr_decoder.length);
      ipctask_LogMessage(&ipc);
      sm_SsrAgLos_Put(&ipc);
      OS_FREE(ssr_decoder.payload);
      memset(&ssr_decoder, 0, sizeof(ssr_decoder));
    }
  } while (-1 != ret);
  return;
}
#endif

/**
 * @brief     Inject SSR Correction stream
 * @param[in] data - stream data buffer
 * @param[in] len - stream data length
 * @param[in] source - stream data service source
 * @return    None
 */
void loc_api_InjectSsrStream(uint8_t* data, uint32_t len, loc_api_ssrServiceSource source)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    return;
  }

  /* Pack a IPC by injected data */
  ipc_t ipc = { 0 };
  if (LOC_API_SSR_SERVICE_SOURCE_NONE == source)
  {
    return;
  }
  else if (LOC_API_SSR_SERVICE_SOURCE_QX == source)
  {
    ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_SSR_STREAM_QX, (uint8_t*)data, len);
    log_ipc(&ipc);
    sm_SsrQianXunStream_Put(&ipc);
  }
  else if (LOC_API_SSR_SERVICE_SOURCE_Gee == source)
  {
    ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_SSR_STREAM_GEE, (uint8_t*)data, len);
    log_ipc(&ipc);
    sm_SsrGeeSpaceStream_Put(&ipc);
  }
  else if (LOC_API_SSR_SERVICE_SOURCE_AG_SRR_LOS == source)
  {
    //loc_api_InjectSsrStream_by_AGssr((uint8_t*)data, len);
  }
}
