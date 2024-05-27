/**@file        gnss_prep.c
 * @brief       previous process for data quality check
 * @details     cycle slip, clock jump repaire
 * @author      houxiaowei
 * @date        2022/5/27
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/5/27   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_prep.h"
#include <math.h>
#include "gnss_common.h"
#include "gnss_def.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "sd_api.h"
#include "mw_alloc.h"
#include "hpp_task.h"
#include "trop_empirical_model.h"
#include "iono_empirical_model.h"

const float g_gf_threslip = 0.05f;
const float g_maxacc_dop = 30.0f;      /* max accel for doppler slip detection (m/s^2) */
const float g_opt_err_dop = 1.0f;      /* measurement error factor: doppler frequency (hz) */
const BOOL g_repaire_clk_jump = TRUE;

/**
 * @brief preprocess: cycle-slip, clock jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 * TODO: after filter, update slip count
 */
void pp_CheckSatSignalQuality(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur)
{
  double tt = tm_GpsTimeDiff(&sat_signal_cur->z_tor, &sat_signal_pre->z_tor);
  if (fabs(tt) < TTOL) {
    return;
  }
  // cycle-slip by GF
  pp_DetectSlipGF(sat_signal_pre, sat_signal_cur);

  // receiver clock jump
  pp_ClkJumpRepaire(sat_signal_pre, sat_signal_cur);

  // cycle-slip by MW
  pp_DetectSlipMW(sat_signal_pre, sat_signal_cur);

  // cycle-slip by DOP
  // pp_DetectSlipDoppler(sat_signal_pre, sat_signal_cur);
}

/**
 * @brief update satellite elevation, use pz_satSigMeasCollect->z_posSolution.xyz
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateSatEleAzi(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  float azi[2] = { 0 };
  double pd_unitVector[3] = { 0 };
  double pd_satEcefRot[3] = { 0 };
  const double* site_pos = pz_satSigMeasCollect->z_positionFix.d_xyz;
  const double* sitella = pz_satSigMeasCollect->z_positionFix.d_lla;
  if (gnss_Norm(site_pos, 3) < FABS_ZEROS)
  {
    return;
  }
  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    gnss_SatelliteMeas_t* pz_satmeas = pz_satSigMeasCollect->pz_satMeas[i];
    if (NULL == pz_satmeas)
    {
      continue;
    }
    const double* sat_pos = pz_satmeas->z_satPosVelClk.d_satPosClk;

    if (gnss_Norm(sat_pos, 3) < FABS_ZEROS)
    {
      continue;
    }
    gnss_satRot(site_pos, sat_pos, pd_satEcefRot);
    gnss_unitVector(site_pos, pd_satEcefRot, pd_unitVector);
    gnss_Satazel(sitella, pd_unitVector, azi);
    pz_satmeas->z_satPosVelClk.f_azimuth = azi[0];
    pz_satmeas->z_satPosVelClk.f_elevation = azi[1];
  }
}

/**
 * @brief empirical value of troposphere, save to satsigmeas
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateTroposphericModel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  /* pos=0, return */
  if (gnss_Norm(pz_satSigMeasCollect->z_positionFix.d_lla, 3) < FABS_ZEROS)
  {
    return;
  }

  /* ztd zhd zwd */
  usingSaasModelCalZenithTropDelay(pz_satSigMeasCollect->z_tor,
    pz_satSigMeasCollect->z_positionFix.d_lla, 0.7, 1,
    &pz_satSigMeasCollect->d_zhd_emp, &pz_satSigMeasCollect->d_zwd_emp);
  pz_satSigMeasCollect->d_ZTD = pz_satSigMeasCollect->d_zhd_emp + pz_satSigMeasCollect->d_zwd_emp;

  /* dry and wet funnction */
  for (int32_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    if (NULL == pz_satSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }
    gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[i];
    pz_meas->d_Tropo = 0.0;

    /* ele=0 */
    if (pz_meas->z_satPosVelClk.f_elevation < FABS_ZEROS)
    {
      continue;
    }

    if (!usingGMFcalculateTropMap(0, pz_satSigMeasCollect->z_positionFix.d_lla, pz_meas->z_satPosVelClk.f_elevation,
      &pz_meas->d_dryMap, &pz_meas->d_wetMap))
    {
      pz_meas->d_Tropo = pz_satSigMeasCollect->d_zhd_emp * pz_meas->d_dryMap
                         + pz_satSigMeasCollect->d_zwd_emp * pz_meas->d_wetMap;
    }
  }
}

/**
 * @brief empirical value of troposphere, save to satsigmeas
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_rtkCorrUpdateTroposphericModel(GnssCorrBlock_t * pz_rtkCorrBlock)
{
  uint16_t w_isat = 0;
  uint32_t q_svindex = 0;
  float f_azel[2] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_ztd = 0.0;
  double d_zhd = 0.0;
  double d_zwd = 0.0;
  double pd_LLA[3] = { 0.0 };
  GnssMeas_t* z_meas = NULL;

  /* pos=0, return */
  if (gnss_Dot(pz_rtkCorrBlock->d_refPosEcef, pz_rtkCorrBlock->d_refPosEcef, 3) < FABS_ZEROS * FABS_ZEROS)
  {
    return;
  }
  gnss_Ecef2Lla(pz_rtkCorrBlock->d_refPosEcef, pd_LLA);
  if (gnss_Dot(pd_LLA, pd_LLA, 3) < FABS_ZEROS * FABS_ZEROS)
  {
    return;
  }
  
  /* ztd zhd zwd */
  usingSaasModelCalZenithTropDelay(pz_rtkCorrBlock->z_Clock.z_gpsTime, pd_LLA, 0.7, 1, &d_zhd, &d_zwd);
  d_ztd = d_zhd + d_zwd;

  for (int16_t w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    pz_rtkCorrBlock->d_trop[w_i] = 0.0;
  }

  /* dry and wet funnction */
  for (int16_t w_i = 0; w_i < pz_rtkCorrBlock->w_measCount; ++w_i)
  {
    z_meas = &pz_rtkCorrBlock->z_meas[w_i];
    if ((q_svindex = gnss_cvt_Svid2SvIndex(z_meas->u_svid,z_meas->u_constellation))
      == ALL_GNSS_SYS_SV_NUMBER)
    {
      continue;
    }

    //get sat pos
    for (w_isat = 0; w_isat < pz_rtkCorrBlock->w_satCount; ++w_isat)
    {
      if (pz_rtkCorrBlock->pz_satPosClk[w_isat].u_constellation == z_meas->u_constellation
        && pz_rtkCorrBlock->pz_satPosClk[w_isat].u_svid == z_meas->u_svid)
      {
        break;
      }
    }
    if (w_isat >= (pz_rtkCorrBlock->w_satCount))
    {
      continue;
    }

    //get ele
    gnss_unitVector(pz_rtkCorrBlock->d_refPosEcef, pz_rtkCorrBlock->pz_satPosClk->d_satPosClk, pd_unitVector);
    gnss_Satazel(pz_rtkCorrBlock->d_refPosEcef, pd_unitVector, f_azel);

    /* ele=0 */
    if (f_azel[1] < 10.0 * DEG2RAD)
    {
      continue;
    }

    pz_rtkCorrBlock->d_trop[q_svindex] = d_ztd / sin(f_azel[1] * RAD2DEG);

  }
}

/**
 * @brief empirical value of ionosphere, save to satsigmeas
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateIonosphericModel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  double d_azel[2] = { 0.0 };
  double d_ion_pra[8] = { 0.0 };
  double d_var = 0.0;
  double d_ion = 0.0;

  /* pos=0, return */
  if (gnss_Norm(pz_satSigMeasCollect->z_positionFix.d_lla, 3) < FABS_ZEROS)
  {
    return;
  }

  for (int32_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    if (NULL == pz_satSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }

    gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[i];
    pz_meas->d_Iono = 0.0;

    /* ele=0 */
    if (pz_meas->z_satPosVelClk.f_elevation < FABS_ZEROS)
    {
      continue;
    }
    d_azel[0] = pz_meas->z_satPosVelClk.f_azimuth;
    d_azel[1] = pz_meas->z_satPosVelClk.f_elevation;

    iono_GetIonoEmpiricalCorrection(pz_satSigMeasCollect->z_tor, d_ion_pra, pz_satSigMeasCollect->z_positionFix.d_lla,
                                    d_azel, IONO_PRO_OPT_BRDC, &d_ion, &d_var);

    double d_frq = gnss_BaseL1Freq(pz_meas->u_constellation);
    if (1.0 == d_frq)
    {
      d_frq = GPS_L1C_FREQ;
    }
    pz_meas->d_Iono = SQR(GPS_L1C_FREQ / d_frq) * d_ion;
  }
}

/**
 * @brief clock jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_ClkJumpRepaire(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur)
{
  uint8_t valid_sat = 0;
  uint8_t num = 0;
  double delta[N_GPS_SV] = { 0 };

  uint8_t jj = gnss_Signal2FreqIdx(C_GNSS_SIG_GPS_L1C);  // frq index
  double lam = wavelength(C_GNSS_SIG_GPS_L1C);


  /* detect / GPS, C_GNSS_SIG_GPS_L1C */
  for (uint16_t i = 0; i < N_GPS_SV; i++) {
    if ((NULL == sat_signal_pre->pz_satMeas[i]) || (NULL == sat_signal_cur->pz_satMeas[i])) {
      continue;
    }

    if ((NULL == sat_signal_cur->pz_satMeas[i]->pz_signalMeas[jj]) ||
      (NULL == sat_signal_pre->pz_satMeas[i]->pz_signalMeas[jj])) {
      continue;
    }
    // GF slip 
    if (0 == (sat_signal_cur->pz_satMeas[i]->pz_signalMeas[jj]->u_slipMask & NON_CYCLE_SLIP_BY_GF))
    {
        continue;
    }

    double p_pre = sat_signal_pre->pz_satMeas[i]->pz_signalMeas[jj]->d_pseudoRange;
    double p_cur = sat_signal_cur->pz_satMeas[i]->pz_signalMeas[jj]->d_pseudoRange;
    double l_pre = sat_signal_pre->pz_satMeas[i]->pz_signalMeas[jj]->d_carrierPhase;
    double l_cur = sat_signal_cur->pz_satMeas[i]->pz_signalMeas[jj]->d_carrierPhase;
    if ((0.0 == p_pre) || (0.0 == p_cur) || (0.0 == l_pre) || (0.0 == l_cur)) {
      continue;
    }

    double d1 = p_pre - p_cur;
    double d2 = (l_pre - l_cur) * lam;

    valid_sat++;
    if (fabs(d1 - d2) > 290000)   // ms clock jump
    {
      delta[num++] = d1 - d2;
    }
  }

  /* update clk jump */
  if ((0 != num) && (num == valid_sat)) {

    double dd = gnss_MeanDouble(delta, num);
    int32_t dd_ms = (int32_t)(dd / CLIGHT * TIME_MSEC);

    if (sat_signal_cur->q_clk_jump != dd_ms) {
      sat_signal_cur->q_clk_jump = dd_ms;
      LOGI(TAG_HPP, "receiver clock jump=%f(ms)\n", dd_ms);
    }
  }

  /* repaire */
  if (sat_signal_cur->q_clk_jump != 0) {
    LOGD(TAG_HPP, "repaire receiver clock jump=%d(ms)\n", sat_signal_cur->q_clk_jump);

    for (uint16_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++) {
      if (NULL == sat_signal_cur->pz_satMeas[i]) {
        continue;
      }
      for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++) {
        if ((sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j] != NULL) &&
          (sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_carrierPhase != 0.0)) {

          double lam_w = wavelength(sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->u_signal);
          if (1.0 == lam_w) continue;
          sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_carrierPhase -= sat_signal_cur->q_clk_jump / TIME_MSEC * CLIGHT / lam_w;
        }
      }
    }
  }
}

 /**
  * @brief geometry-free phase measurement
  * @param[in] pz_meas satellite measurement
  * @param[in] u_frqidx frequency index
  * @return
  */
double pp_MeasGF(const gnss_SatelliteMeas_t* pz_meas, const uint8_t u_frqidx)
{
  double gf = .0;
  uint8_t sigidx1 = 0;
  uint8_t sigidx2 = u_frqidx;

  if ((pz_meas->pz_signalMeas[sigidx1] != NULL) && (pz_meas->pz_signalMeas[sigidx2] != NULL))
  {
    double freq1 = CLIGHT / wavelength(pz_meas->pz_signalMeas[sigidx1]->u_signal);
    double freq2 = CLIGHT / wavelength(pz_meas->pz_signalMeas[sigidx2]->u_signal);

    if ((freq1 != freq2) && (freq1 != CLIGHT) && (freq2 != CLIGHT))
    {
      double l1 = pz_meas->pz_signalMeas[sigidx1]->d_carrierPhase;
      double l2 = pz_meas->pz_signalMeas[sigidx2]->d_carrierPhase;

      if ((l1 != 0.0) && (l2 != 0.0))
      {
        gf = (l1 / freq1 - l2 / freq2) * CLIGHT;
      }
    }
  }

  return gf;
}


/**
 * @brief detect freq cycle slip by geometry free phase jump, combine: L2 or L5 with L1
 * @param[in,out] pz_meas currert meas
 * @param[in] u_frqidx 1 ~ (MAX_GNSS_SIGNAL_FREQ-1)
 */
void pp_DetectSlipGFFrq(gnss_SatelliteMeas_t* pz_meas, const uint8_t u_frqidx)
{
  float f_g0 = 0.0f;
  float f_g1 = 0.0f;

  f_g0 = pz_meas->f_gf[u_frqidx];
  pz_meas->f_gf[u_frqidx] = 0;

  /* NULL== signal || frq error */
  if ((NULL == pz_meas->pz_signalMeas[0]) || (u_frqidx >= MAX_GNSS_SIGNAL_FREQ) ||
      (NULL == pz_meas->pz_signalMeas[u_frqidx]))
  {
    return;
  }

  /* gf meas */
  f_g1 = (float) pp_MeasGF(pz_meas, u_frqidx);
  if (0.0 == f_g1)
  {
    return;
  }
  pz_meas->f_gf[u_frqidx] = f_g1;

  /* slip */
  if ((0.0 != f_g0) && (fabsf(f_g1 - f_g0) <= g_gf_threslip))
  {
    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
    {
      if (NULL == pz_meas->pz_signalMeas[j])
      {
        continue;
      }
      pz_meas->pz_signalMeas[j]->u_slipMask |= NON_CYCLE_SLIP_BY_GF;
    }
  }
  else
  {
    char sid[8] = {0};
    svid_SatString(pz_meas->u_svid, pz_meas->u_constellation, sid);
    LOGI(TAG_HPP, "slip gf: sat=%s frq:1+%d gf0=%8.3f gf1=%8.3f\n", sid, u_frqidx + 1, f_g0, f_g1);
  }
}

/**
 * @brief GF, detect cycle slip by geometry free phase jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipGF(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur)
{
  for (uint16_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++)
  {
    if ((NULL == sat_signal_pre->pz_satMeas[i]) || (NULL == sat_signal_cur->pz_satMeas[i]))
    {
      continue;
    }
    gnss_SatelliteMeas_t* pz_meas = sat_signal_cur->pz_satMeas[i];
    const gnss_SatelliteMeas_t* pz_meas_p = sat_signal_pre->pz_satMeas[i];
    memcpy(pz_meas->f_gf, pz_meas_p->f_gf, sizeof(pz_meas->f_gf));

    /* L1-L2 / L1-L5 /.... */
    for (uint8_t j = C_GNSS_FREQ_TYPE_L2; j < C_GNSS_FREQ_TYPE_MAX; ++j)
    {
      pp_DetectSlipGFFrq(pz_meas, j);
    }
  }
}

/**
 * @brief Melbourne-Wubbena linear combination
 * @param[in] pz_meas currert satellite measurement
 * @param u_frqidx frequency index
 * @return
 */
double pp_MeasMW(const gnss_SatelliteMeas_t* pz_meas, const uint8_t u_frqidx)
{
  double mw = .0;
  uint8_t sigidx1 = 0;
  uint8_t sigidx2 = u_frqidx;
  if ((pz_meas->pz_signalMeas[sigidx1] != NULL) && (pz_meas->pz_signalMeas[sigidx2] != NULL))
  {
    double freq1 = CLIGHT / wavelength(pz_meas->pz_signalMeas[sigidx1]->u_signal);
    double freq2 = CLIGHT / wavelength(pz_meas->pz_signalMeas[sigidx2]->u_signal);

    if ((freq1 != freq2) && (freq1 != CLIGHT) && (freq2 != CLIGHT))
    {
      double l1 = pz_meas->pz_signalMeas[sigidx1]->d_carrierPhase;
      double l2 = pz_meas->pz_signalMeas[sigidx2]->d_carrierPhase;
      double p1 = pz_meas->pz_signalMeas[sigidx1]->d_pseudoRange;
      double p2 = pz_meas->pz_signalMeas[sigidx2]->d_pseudoRange;
      if ((p1 != 0.0) && (p2 != 0.0) && (l1 != 0.0) && (l2 != 0.0))
      {
        double lam_wl = CLIGHT / (freq1 - freq2);
        mw = (l1 - l2) - (freq1 * p1 + freq2 * p2) / (freq1 + freq2) / (lam_wl);
      }
    }
  }

  return mw;
}

/**
 * @brief MW, combine: L2 or L5 with L1
 * @param[in,out] pz_meas currert satellite measurement
 * @param[in] u_frqidx 1 ~ (MAX_GNSS_SIGNAL_FREQ-1)
 */
void pp_DetectSlipMWFrq(gnss_SatelliteMeas_t* pz_meas, const uint8_t u_frqidx)
{
  uint8_t u_slip = 0;

  /* NULL==L1 signal || frq error */
  if ((NULL == pz_meas->pz_signalMeas[0]) || (u_frqidx >= MAX_GNSS_SIGNAL_FREQ) ||
      (NULL == pz_meas->pz_signalMeas[u_frqidx]))
  {
    return;
  }

  /* mw meas */
  float w1 = (float) pp_MeasMW(pz_meas, u_frqidx);
  if (0.0f == w1)
  {
    return;
  }
  pz_meas->f_mw[u_frqidx] = w1;

  if ((0.0 == pz_meas->f_mw_avg[u_frqidx]) && (0 == pz_meas->q_mw_cnt[u_frqidx]))
  {
    pz_meas->q_mw_cnt[u_frqidx] = 1;
    pz_meas->f_mw_avg[u_frqidx] = w1;
    pz_meas->f_mw_std[u_frqidx] = 0.0f;
    return;
  }
  pz_meas->q_mw_cnt[u_frqidx]++;
  float mw_de = w1 - pz_meas->f_mw_avg[u_frqidx];
  float mw_avg = pz_meas->f_mw_avg[u_frqidx] + mw_de / ((float) pz_meas->q_mw_cnt[u_frqidx]);
  float mw_std = pz_meas->f_mw_std[u_frqidx] + (mw_de * mw_de - pz_meas->f_mw_std[u_frqidx]) / (float) (pz_meas->q_mw_cnt[u_frqidx]);

  float sigma = sqrtf(pz_meas->f_mw_std[u_frqidx]);
  if (2 == pz_meas->q_mw_cnt[u_frqidx])
  {
    if (fabsf(mw_de) >= 10.0)
    {
      u_slip = 1;
    }
  }
  else if (pz_meas->q_mw_cnt[u_frqidx] > 2)
  {
    if (sigma > 5.0 || (fabsf(mw_de) >= 4 * sigma && fabsf(mw_de) > 1.0f))
    {
      u_slip = 1;
    }
  }
  if (1 == u_slip)
  {
    char sid[8] = {0};
    svid_SatString(pz_meas->u_svid, pz_meas->u_constellation, sid);
    LOGI(TAG_HPP, "slip mw: sat=%s frq:1+%d mw0=%8.3f mw1=%8.3f threshod=%8.3f\n", sid, u_frqidx + 1,
         pz_meas->f_mw_avg[u_frqidx], w1, 4 * sigma);

    /* clean mw info */
    pz_meas->q_mw_cnt[u_frqidx] = 1;
    pz_meas->f_mw_avg[u_frqidx] = w1;
    pz_meas->f_mw_std[u_frqidx] = 0.0f;
  }
  else
  {
    pz_meas->f_mw_avg[u_frqidx] = mw_avg;
    pz_meas->f_mw_std[u_frqidx] = mw_std;
    /* set slip flag */
    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
    {
      if (NULL == pz_meas->pz_signalMeas[j])
      {
        continue;
      }
      pz_meas->pz_signalMeas[j]->u_slipMask |= NON_CYCLE_SLIP_BY_MW;
    }
  }
}

/**
 * @brief MW, detect slip by Melbourne-Wubbena linear combination jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipMW(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur)
{
  for (uint16_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++)
  {
    if ((NULL == sat_signal_pre->pz_satMeas[i]) || (NULL == sat_signal_cur->pz_satMeas[i]))
    {
      continue;
    }
    gnss_SatelliteMeas_t* pz_meas = sat_signal_cur->pz_satMeas[i];
    const gnss_SatelliteMeas_t* pz_meas_p = sat_signal_pre->pz_satMeas[i];
    memcpy(pz_meas->q_mw_cnt, pz_meas_p->q_mw_cnt, sizeof(pz_meas->q_mw_cnt));
    memcpy(pz_meas->f_mw_avg, pz_meas_p->f_mw_avg, sizeof(pz_meas->f_mw_avg));
    memcpy(pz_meas->f_mw_std, pz_meas_p->f_mw_std, sizeof(pz_meas->f_mw_std));

    /* L1-L2 / L1-L5 /.... */
    for (uint8_t j = C_GNSS_FREQ_TYPE_L2; j < C_GNSS_FREQ_TYPE_MAX; ++j)
    {
      pp_DetectSlipMWFrq(pz_meas, j);
    }
  }
}

/**
 * @brief DOPPLER, ddetect cycle slip by doppler and phase difference
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipDoppler(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur)
{

  for (uint16_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++) {
    if ((NULL == sat_signal_pre->pz_satMeas[i]) ||
      (NULL == sat_signal_cur->pz_satMeas[i]))
    {
      continue;
    }

    double tt = tm_GpsTimeDiff(&sat_signal_cur->z_tor, &sat_signal_pre->pz_satMeas[i]->z_tot);

    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
    {
      if ((NULL == sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]) ||
        (NULL == sat_signal_pre->pz_satMeas[i]->pz_signalMeas[j]))
      {
        continue;
      }

      if ((0 == sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_carrierPhase) ||
        (0 == sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_doppler))
      {
        continue;
      }

      double dopd = tt * sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_doppler;

      double phased = sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->d_carrierPhase -
        sat_signal_pre->pz_satMeas[i]->pz_signalMeas[j]->d_carrierPhase;

      /* threshold (cycle) */
      double lam = wavelength(sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->u_signal);
      if (1.0 == lam) {
        continue;
      }
      dopd = dopd / lam;

      double thres = g_maxacc_dop * tt * tt / 2.0 / lam + g_opt_err_dop * fabs(tt) * 4.0;

      if (fabs(dopd - phased) <= thres) {
        sat_signal_cur->pz_satMeas[i]->pz_signalMeas[j]->u_slipMask |= NON_1CYCLE_SLIP_BY_DOPPLER;
      }
      else{
        char sid[8] = { 0 };
        svid_SatString(sat_signal_cur->pz_satMeas[i]->u_svid, sat_signal_cur->pz_satMeas[i]->u_constellation, sid);
        LOGI(TAG_HPP, "slip doppler: sat=%s dopd=%8.3f, phased=%8.3f, thres=%.3f\n", sid, dopd, phased, thres);
      }
    }
  }
}


