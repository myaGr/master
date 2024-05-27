/**@file        dcp_common.c
 * @brief       define the interface of DCP model
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/10/25  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "dcp_common.h"
#include "sd_api.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "mw_log.h"
#include "seq_kalman_float.h"

 /**
  * @brief initialize the dcp_interRecordInfo_t
  * @param[out] pz_dcpInfo, initilize the calculated information used by TDCP or TDPR algorithm
  * @return void
  */
void dcp_initDcpInterInfo(dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  pz_dcpInfo->u_posObainType = GNSS_DCP_POS_INVALID;
  pz_dcpInfo->u_paraNum = 0;
  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pu_slipTag[u_i] = DCP_NON_DETECT;
    pz_dcpInfo->pf_wP[u_i] = 1.0;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_dcpInfo->pf_rcvClkDrift[u_i] = 0.0;
  }
  pz_dcpInfo->f_deltaTime = 0.0;
  pz_dcpInfo->f_sigma0 = 0.0;
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_X[u_i] = pz_dcpInfo->pf_deltaX[u_i] = 0.0;
  }
  for (u_i = 0; u_i < TD_MAX_Q_ELE; ++u_i)
  {
    pz_dcpInfo->pf_Q[u_i] = 0.0;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_dcpInfo->pd_curXyz[u_i] = 0.0;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_dcpInfo->u_goodQualityObsNum[u_i] = 0;
  }
  return;
}

/**
 * @brief remove the constellation by the index
 * @param[in]  u_deleteSysIndex,the will be deleted constellation index
 * @return constellation used in the algorithm
 */
algo_useConstellation dcp_removeSysDeleteByIndex(uint8_t u_deleteSysIndex)
{
  algo_useConstellation z_deletedSys = ALGO_NON_SYS;
  if (0 == u_deleteSysIndex)
  {
    z_deletedSys = ALGO_GAL_SYS;
  }
  else if (1 == u_deleteSysIndex)
  {
    z_deletedSys = ALGO_QZS_SYS;
  }
  else if (2 == u_deleteSysIndex)
  {
    z_deletedSys = ALGO_BDS_SYS;
  }
  else if (3 == u_deleteSysIndex)
  {
    z_deletedSys = ALGO_GPS_SYS;
  }
  return z_deletedSys;
}

/**
 * @brief initilize the struct used by sequential kalman filter,the type is float
 * @param[in]   w_paraNum               the total number of parameter
 * @param[out]  pz_seqKalmanFloatVar,   the struct used by sequential kalman filter,the type is float
 * @return      1 represent obtained memory success and 0 represent obtained memory failure
 */
uint8_t dcp_initSeqKalmanFloatVar(uint16_t w_paraNum, SeqKalmanFloatVar_t* pz_seqKalmanFloatVar)
{
  uint8_t u_status = 1;
  if (NULL != pz_seqKalmanFloatVar)
  {
    pz_seqKalmanFloatVar->w_paraNum = w_paraNum;
    pz_seqKalmanFloatVar->w_n = 0;
    pz_seqKalmanFloatVar->f_Z = 0.0;
    pz_seqKalmanFloatVar->f_R = 0.0;
    pz_seqKalmanFloatVar->f_pZ = 0.0;
    pz_seqKalmanFloatVar->f_zL = 0.0;
    pz_seqKalmanFloatVar->f_rL = 0.0;
    pz_seqKalmanFloatVar->f_res = 0.0;
    pz_seqKalmanFloatVar->pf_h = (float*)OS_MALLOC_FAST(w_paraNum * sizeof(float));
    pz_seqKalmanFloatVar->pf_hP = (float*)OS_MALLOC_FAST(w_paraNum * sizeof(float));
    pz_seqKalmanFloatVar->pf_k = (float*)OS_MALLOC_FAST(w_paraNum * sizeof(float));
    pz_seqKalmanFloatVar->pw_L = (uint16_t*)OS_MALLOC_FAST(w_paraNum * sizeof(uint16_t));
    if (any_Ptrs_Null(4, pz_seqKalmanFloatVar->pf_h, pz_seqKalmanFloatVar->pf_hP, pz_seqKalmanFloatVar->pf_k, pz_seqKalmanFloatVar->pw_L))
    {
      OS_FREE(pz_seqKalmanFloatVar->pf_h);
      OS_FREE(pz_seqKalmanFloatVar->pf_hP);
      OS_FREE(pz_seqKalmanFloatVar->pf_k);
      OS_FREE(pz_seqKalmanFloatVar->pw_L);
      u_status = 0;
    }
  }
  return u_status;
}

/**
 * @brief deinitilize the struct used by sequential kalman filter,the type is float
 * @param[out]  pz_seqKalmanFloatVar,   the struct used by sequential kalman filter,the type is float
 * @return      void
 */
void dcp_deinitSeqKalmanFloatVar(SeqKalmanFloatVar_t* pz_seqKalmanFloatVar)
{
  if (NULL != pz_seqKalmanFloatVar)
  {
    if (NULL != pz_seqKalmanFloatVar->pf_h)
    {
      OS_FREE(pz_seqKalmanFloatVar->pf_h);
    }

    if (NULL != pz_seqKalmanFloatVar->pf_hP)
    {
      OS_FREE(pz_seqKalmanFloatVar->pf_hP);
    }

    if (NULL != pz_seqKalmanFloatVar->pf_k)
    {
      OS_FREE(pz_seqKalmanFloatVar->pf_k);
    }

    if (NULL != pz_seqKalmanFloatVar->pw_L)
    {
      OS_FREE(pz_seqKalmanFloatVar->pw_L);
    }
  }
  return;
}

/**
 * @brief get the factor of deweight of the robust estimation
 * @param[in]  f_k0 the divide point of keep and deweight
 * @param[in]  f_k1, the divide point of deweight and reject
 * @param[in]  f_sigma0, prior sigma
 * @param[in]  f_res, the post residual of observation
 * @return the factor of deweight of the robust estimation
 */
float dcp_getWp(float f_k0, float f_k1, float f_sigma0, float f_res)
{
  float f_wp = 1.0f;
  float f_vDivideSigma = fabsf(f_res / f_sigma0);
  float f_temp = 0.0f;
  if (f_vDivideSigma <= f_k0)
  {
    f_wp = 1.0f;
  }
  else if (f_vDivideSigma <= f_k1)
  {
    f_temp = (f_k1 - f_vDivideSigma) / (f_k1 - f_k0);
    f_wp = f_k0 / f_vDivideSigma * f_temp * f_temp;
  }
  else
  {
    f_wp = 1.0e-12f;
  }
  return f_wp;
}

/**
 * @brief fillup the current position solution struct by the previous epoch
 * @param[in] pz_prePosSol,the previous position information
 * @param[out] pz_curPosSol,the current position information
 * @return void
 */
void dcp_fillupPosSolUsingPreEpoch(const gnss_PositionFix_t* pz_prePosSol, gnss_PositionFix_t* pz_curPosSol)
{
  uint8_t u_i = 0;
  if ((NULL == pz_prePosSol) || (NULL == pz_curPosSol))
  {
    return;
  }

  *pz_curPosSol = *pz_prePosSol;
  pz_curPosSol->w_size = sizeof(gnss_PositionFix_t);
  pz_curPosSol->z_rtk.f_age = 0.0f;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_curPosSol->f_velXyz[u_i] = 0.0f;
    pz_curPosSol->f_velEnu[u_i] = 0.0f;
    pz_curPosSol->f_posXyzUnc[u_i] = 0.0f;
    pz_curPosSol->f_velEnuUnc[u_i] = 0.0f;
    pz_curPosSol->d_xyz[u_i] = 0.0;
    pz_curPosSol->d_lla[u_i] = 0.0;
  }
  pz_curPosSol->d_clockBias = 0.0;
  pz_curPosSol->d_clockDrift = 0.0;
  pz_curPosSol->u_fixSource = FIX_SOURCE_DCP; // DCP SOURCE solution
  pz_curPosSol->u_fixFlag = pz_prePosSol->u_fixFlag;

  memcpy(&pz_curPosSol->z_rtk, &(pz_prePosSol->z_rtk), sizeof(gnss_PositionFix_RTK_t));
  memcpy(&pz_curPosSol->z_dops, &pz_prePosSol->z_dops, sizeof(gnss_PositionFix_Dops_t));
  memcpy(&pz_curPosSol->z_SvStatus, &pz_prePosSol->z_SvStatus, sizeof(gnss_PositionFix_SvStatus_t));
  return;
}

/**
 * @brief judge whether going the TDCP or TDPR routeline
 * @param[in] pu_preSvUsed,  the number of satellites used by previous epoch
 * @param[in] pu_flags,      the type of position solution for the previous epoch
 * @param[in] pd_xyz[3],     the ECEF position for the previous epoch
 * @param[in] pf_deltaTime,  the interval between the previous epoch and current epoch
 * @return 1 represent going the TDCP or TDPR routeline, 0 represent don't go the TDCP or TDPR routeline
 */
uint8_t dcp_checkGoingTDroute(const uint8_t* pu_preSvUsed, const uint8_t* pu_flags, const double pd_xyz[3], const float* pf_deltaTime)
{
  uint8_t u_goingTDroute = 1;
  if (/**pu_preSvUsed <= 4 || GNSS_FIX_FLAG_INVALID == *pu_flags
    ||*/ fabs(*pf_deltaTime) >= 30.0 || fabs(*pf_deltaTime) < 1.0e-3
    || (fabs(pd_xyz[0]) < 1.0e-3 && fabs(pd_xyz[1]) < 1.0e-3 && fabs(pd_xyz[2]) < 1.0e-3))
  {
    u_goingTDroute = 0;
  }
  return u_goingTDroute;
}

/**
 * @brief find the index of satellite data to get the satellite position and clock
 * @param[in] u_sys,GNSS constellation
 * @param[in] u_prn,satellite index of the system
 * @param[in] pz_dcpMeas, the data used by TDCP,include current epoch and previous epoch
 * @return int16_t,index of satellite data
 */
int16_t dcp_findSatDataIndex(uint8_t u_sys, uint8_t u_prn, const gnss_TdcpMeasBlock_t* pz_dcpMeas)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  for (w_i = 0; w_i < (pz_dcpMeas->w_satNum); ++w_i)
  {
    if (pz_dcpMeas->pz_satInfo[w_i].u_constellation == u_sys
      && pz_dcpMeas->pz_satInfo[w_i].u_svid == u_prn)
    {
      w_index = w_i;
      break;
    }
  }
  return w_index;
}

/**
 * @brief fillup the satellite postion,velocity and satellite clock and drift field
 * @param[out] pz_curDcpMeas,TDCP measure block of current epoch
 * @return number of sat info, 0 = failed
 */
uint16_t dcp_fillupCurSatInfo(const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint16_t w_iMeas = 0;
  uint16_t w_iSat = 0;
  uint16_t w_satIndex = 0;
  uint8_t u_svid = 0;
  uint8_t u_i = 0;
  int16_t pre_sat_idx = -1;
  GpsTime_t z_tot;
  BOOL q_bSuccess = FALSE;
  gnss_SatPosVelClk_t* pz_satPosVelClk = OS_MALLOC_FAST(sizeof(gnss_SatPosVelClk_t));
  if (any_Ptrs_Null(1, pz_satPosVelClk))
  {
    return 0;
  }

  pz_curDcpMeas->w_satNum = 0;
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    if ((pz_curDcpMeas->w_satNum) >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    if ((pz_curDcpMeas->pz_meas[w_iMeas].d_pseudoRange) < FABS_ZEROS)
    {
      continue;
    }
    for (w_iSat = 0; w_iSat < (pz_curDcpMeas->w_satNum); ++w_iSat)
    {
      if (((pz_curDcpMeas->pz_satInfo[w_iSat].u_constellation) == (pz_curDcpMeas->pz_meas[w_iMeas].u_constellation))
        && ((pz_curDcpMeas->pz_satInfo[w_iSat].u_svid) == (pz_curDcpMeas->pz_meas[w_iMeas].u_svid)))
      {
        break;
      }
    }
    if (w_iSat < (pz_curDcpMeas->w_satNum))
    {
      continue;
    }
    w_satIndex = (pz_curDcpMeas->w_satNum);
    gnss_DcpSatInfo_t* pz_satinfo = pz_curDcpMeas->pz_satInfo + w_satIndex;
    pz_satinfo->u_valid = FALSE;
    pz_satinfo->u_isCalculateDist = FALSE;
    pz_satinfo->u_constellation = pz_curDcpMeas->pz_meas[w_iMeas].u_constellation;
    pz_satinfo->u_svid = pz_curDcpMeas->pz_meas[w_iMeas].u_svid;
    z_tot = (pz_curDcpMeas->z_obsTime);
    tm_GpstTimeAdd(&z_tot, -(pz_curDcpMeas->pz_meas[w_iMeas].d_pseudoRange) / CLIGHT);

    /* get satellite pos clk */
    memset(pz_satPosVelClk, 0, sizeof(gnss_SatPosVelClk_t));
    q_bSuccess = sd_api_SatPosVelClk_Get(pz_satinfo->u_constellation, pz_satinfo->u_svid, &z_tot, pz_satPosVelClk);
    if (q_bSuccess)
    {
      pz_satinfo->q_iode = pz_satPosVelClk->q_iode;
      for (u_i = 0; u_i < 4; ++u_i)
      {
        pz_satinfo->pd_satPosClk[u_i] = pz_satPosVelClk->d_satPosClk[u_i];
        pz_satinfo->pd_satVelClk[u_i] = pz_satPosVelClk->d_satVelClk[u_i];
      }
      pz_satinfo->u_valid = TRUE;
      pre_sat_idx = dcp_findSatDataIndex(pz_satinfo->u_constellation, pz_satinfo->u_svid, pz_preDcpMeas);
      if (pre_sat_idx != -1)
      {
        pz_satinfo->f_elevation = pz_preDcpMeas->pz_satInfo[pre_sat_idx].f_elevation;
        pz_satinfo->f_azimuth = pz_preDcpMeas->pz_satInfo[pre_sat_idx].f_azimuth;
      }
      ++(pz_curDcpMeas->w_satNum);
    }
  }
  OS_FREE(pz_satPosVelClk);
  return pz_curDcpMeas->w_satNum;
}

/**
 * @brief update previous meas satellite information if CURRENT iode not equal PREVIOUS
 * @param[in] pz_curDcpMeas
 * @param[in,out] pz_preDcpMeas
 */
void dcp_updatePreSatInfo(const gnss_TdcpMeasBlock_t* pz_curDcpMeas, gnss_TdcpMeasBlock_t* pz_preDcpMeas)
{
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_svid = 0;
  uint8_t u_sat_num = 0;
  for (int i = 0; i < pz_curDcpMeas->w_satNum; ++i)
  {
    const gnss_DcpSatInfo_t* pz_cur_satinfo = (pz_curDcpMeas->pz_satInfo) + i;
    u_constellation = pz_cur_satinfo->u_constellation;
    u_svid = pz_cur_satinfo->u_svid;
    if (FALSE == pz_cur_satinfo->u_valid)
    {
      continue;
    }
    int16_t idx_pre = dcp_findSatDataIndex(u_constellation, u_svid, pz_preDcpMeas);
    if (-1 == idx_pre)
    {
      continue;
    }
    gnss_DcpSatInfo_t* pz_pre_satinfo = (pz_preDcpMeas->pz_satInfo) + idx_pre;
    if (pz_pre_satinfo->q_iode != pz_cur_satinfo->q_iode)
    {
      gnss_SatPosVelClk_t pz_satPosVelClk = { 0 };
      GpsTime_t tot = pz_preDcpMeas->z_obsTime;
      if (gnss_CalGnssMeasTot(pz_preDcpMeas->pz_meas, (uint8_t)pz_preDcpMeas->w_measNum, u_constellation, u_svid, &tot) &&
        sd_api_SatPositionFromBrdc(&tot, u_constellation, u_svid, pz_cur_satinfo->q_iode, TRUE, &pz_satPosVelClk))
      {
        pz_pre_satinfo->u_valid = TRUE;
        pz_pre_satinfo->u_isCalculateDist = TRUE;
        pz_pre_satinfo->q_iode = pz_satPosVelClk.q_iode;
        for (int j = 0; j < 4; ++j)
        {
          pz_pre_satinfo->pd_satPosClk[j] = pz_satPosVelClk.d_satPosClk[j];
          pz_pre_satinfo->pd_satVelClk[j] = pz_satPosVelClk.d_satVelClk[j];
        }
        pz_pre_satinfo->d_sat2SiteDist = gnss_unitVector(pz_preDcpMeas->z_posSol.d_xyz, pz_pre_satinfo->pd_satPosClk,
          pz_pre_satinfo->pd_site2SatUnit);
      }
      else
      {
        u_sat_num++;
      }
    }
    else
    {
      u_sat_num++;
    }
  }
  LOGI(TAG_DCP, "Sat info match suc: %d\n", u_sat_num);
}

/**
 * @brief find the index observation in the previous epoch
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in] u_curSys, GNSS constellation
 * @param[in] u_curPrn, satellite index of the system
 * @param[in] e_curFreqType, frequency type of satellite
 * @param[out] pu_preIndex, the index of observation in previous epoch
 * @return 1 represent find target observations success, otherwise, 0 represent find target observations failure
 */
uint8_t dcp_findMeasPreEpoch(const gnss_TdcpMeasBlock_t* pz_preDcpMeas, uint8_t u_curSys, uint8_t u_curPrn, gnss_FreqType e_curFreqType, uint8_t* pu_preIndex)
{
  uint8_t u_status = 0;
  uint16_t w_imeas = 0;
  gnss_FreqType e_preFreqType = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_meas = NULL;
  *pu_preIndex = 0;
  for (w_imeas = 0; w_imeas < pz_preDcpMeas->w_measNum; ++w_imeas)
  {
    pz_meas = pz_preDcpMeas->pz_meas + w_imeas;
    if ((pz_meas->u_constellation) != u_curSys || (pz_meas->u_svid) != u_curPrn)
    {
      continue;
    }
    e_preFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)(pz_meas->u_signal));
    if (e_preFreqType == e_curFreqType)
    {
      *pu_preIndex = (uint8_t)w_imeas;
      u_status = 1;
      break;
    }
  }
  return u_status;
}

/**
 * @brief judge whether the time difference observations is valid
 * @param[in] pz_meas is the pointer of current epoch observations
 * @param[in] pz_preMeas is the pointer of previous epoch observations
 * @return uint8_t,0 represent the time difference observations is invalid and 1 represent valid
 */
uint8_t dcp_WhetherTDobsValid(const GnssMeas_t* pz_meas, const GnssMeas_t* pz_preMeas)
{
  uint8_t u_obsValid = 1;
  if ((fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    && (fabs(pz_meas->d_doppler) < 1.0e-3 || fabs(pz_preMeas->d_doppler) < 1.0e-3)
    && ((pz_meas->d_pseudoRange) < 1.0e-6 || (pz_preMeas->d_pseudoRange) < 1.0e-6))
  {
    u_obsValid = 0;
  }
  return u_obsValid;
}

/**
 * @brief judge the valid of satellite position
 * @param[in] pd_satPosClk,the satellite position and clock
 * @return uint8_t,0 represent the satellite position and clock is invalid and 1 reprenset valid
 */
uint8_t dcp_whetherSatInfoValid(const double pd_satPosClk[4])
{
  uint8_t u_satInfoValid = 1;
  uint8_t u_i = 0;
  for (u_i = 0; u_i < 4; ++u_i)
  {
    if (fabs(pd_satPosClk[u_i]) < FABS_ZEROS)
    {
      u_satInfoValid = 0;
      break;
    }
  }
  return u_satInfoValid;
}

/**
 * @brief gain the information of site to one satellite, include distance and unit vector
 * @param[in] pd_siteCoor[3], site coordinate in ECEF
 * @param[out] pd_curSatDcpInfo,the satellite information of current epoch used by DCP
 * @param[out] pd_preSatDcpInfo,the satellite information of previous epoch used by DCP
 * @return void
 */
void dcp_calSite2SatUnitDist(const double pd_siteCoor[3], gnss_DcpSatInfo_t* pd_curSatDcpInfo, gnss_DcpSatInfo_t* pd_preSatDcpInfo)
{
  uint8_t u_k = 0;
  double pd_e[3] = { 0.0 };
  double pd_baseE[3] = { 0.0 };
  double d_range = 0.0;
  double d_baseRange = 0.0;
  const double* pd_curSatPosClk = pd_curSatDcpInfo->pd_satPosClk;
  const double* pd_preSatPosClk = pd_preSatDcpInfo->pd_satPosClk;
  if (!dcp_whetherSatInfoValid(pd_curSatPosClk) || !dcp_whetherSatInfoValid(pd_preSatPosClk))
  {
    return;
  }
  if (1 == (pd_curSatDcpInfo->u_isCalculateDist) && 1 == (pd_preSatDcpInfo->u_isCalculateDist))
  {
    return;
  }
  for (u_k = 0; u_k < 3; u_k++)
  {
    pd_e[u_k] = pd_curSatPosClk[u_k] - pd_siteCoor[u_k];
  }
  d_range = gnss_Norm(pd_e, 3);
  for (u_k = 0; u_k < 3; u_k++)
  {
    pd_e[u_k] /= d_range;
    pd_curSatDcpInfo->pd_site2SatUnit[u_k] = pd_e[u_k];
  }
  for (u_k = 0; u_k < 3; u_k++)
  {
    pd_baseE[u_k] = pd_preSatPosClk[u_k] - pd_siteCoor[u_k];
  }
  d_baseRange = gnss_Norm(pd_baseE, 3);
  pd_curSatDcpInfo->d_sat2SiteDist = d_range;
  pd_preSatDcpInfo->d_sat2SiteDist = d_baseRange;
  pd_curSatDcpInfo->u_isCalculateDist = 1;
  pd_preSatDcpInfo->u_isCalculateDist = 1;
  return;
}

/**
 * @brief gain the information of site to all satellite, include distance and unit vector,
 *        ignore the correction of earth rotion, because it very small for the epoch difference observation
 * @param[in]   pd_siteCoor the coordinate of site in ECEF
 * @param[out]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]  pz_curDcpMeas TDCP measure block of current epoch
 * @return void
 */
void dcp_gainSite2SatInfo(const double pd_siteCoor[3], gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint8_t u_imeas = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_preIndex = 0;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  for (u_i = 0; u_i < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_i)
  {
    pz_preDcpMeas->pz_satInfo[u_i].u_isCalculateDist = 0;
    pz_curDcpMeas->pz_satInfo[u_i].u_isCalculateDist = 0;
    pz_curDcpMeas->pz_satInfo[u_i].d_sat2SiteDist = 0.0;
    for (u_j = 0; u_j < 3; ++u_j)
    {
      pz_curDcpMeas->pz_satInfo[u_i].pd_site2SatUnit[u_j] = 0.0;
    }
  }
  for (u_imeas = 0; u_imeas < pz_curDcpMeas->w_measNum; ++u_imeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + u_imeas;
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (C_GNSS_FREQ_TYPE_MAX == e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (!dcp_WhetherTDobsValid(pz_meas, pz_preMeas))
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid)
    {
      continue;
    }
    dcp_calSite2SatUnitDist(pd_siteCoor, &(pz_curDcpMeas->pz_satInfo[w_curSatIndex]), &(pz_preDcpMeas->pz_satInfo[w_preSatIndex]));
  }
  return;
}
/**
 * @brief GF corse detect cycle slip using the previous epoch and current observations
 * @param[in]  e_baseFreqType the first frequency type
 * @param[in]  e_twinFreqType the twin frequency type
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[out] pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @return void
 */
void dcp_gfCorseDetectSlip(gnss_FreqType e_firstFreqType, gnss_FreqType e_twinFreqType, const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint16_t w_imeas1 = 0;
  uint16_t w_imeas2 = 0;
  uint8_t u_preIndex1 = 0;
  uint8_t u_preIndex2 = 0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_freqType1 = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType e_freqType2 = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_meas1 = NULL;
  const GnssMeas_t* pz_meas2 = NULL;
  const GnssMeas_t* pz_preMeas1 = NULL;
  const GnssMeas_t* pz_preMeas2 = NULL;
  double d_wave1 = 0.0;
  double d_wave2 = 0.0;
  double d_deltaGf = 0.0;
  for (w_imeas1 = 0; w_imeas1 < pz_curDcpMeas->w_measNum; ++w_imeas1)
  {
    pz_meas1 = pz_curDcpMeas->pz_meas + w_imeas1;
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas1->u_constellation))
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas1->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_freqType1 = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas1->u_signal);
    if (e_firstFreqType != e_freqType1)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas1->u_constellation, pz_meas1->u_svid, e_freqType1, &u_preIndex1))
    {
      continue;
    }
    for (w_imeas2 = 0; w_imeas2 < pz_curDcpMeas->w_measNum; ++w_imeas2)
    {
      pz_meas2 = pz_curDcpMeas->pz_meas + w_imeas2;
      e_freqType2 = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas2->u_signal);
      if (C_GNSS_FREQ_TYPE_MAX == e_freqType2 || e_freqType2 == e_freqType1 || e_freqType2 != e_twinFreqType)
      {
        continue;
      }
      if ((pz_meas1->u_constellation) != (pz_meas2->u_constellation)
        || (pz_meas1->u_svid != pz_meas2->u_svid))
      {
        continue;
      }
      if (fabs(pz_meas1->d_carrierPhase) < 1.0e-6
        || fabs(pz_meas2->d_carrierPhase) < 1.0e-6)
      {
        continue;
      }
      if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas2->u_constellation, pz_meas2->u_svid, e_freqType2, &u_preIndex2))
      {
        continue;
      }
      pz_preMeas1 = pz_preDcpMeas->pz_meas + u_preIndex1;
      pz_preMeas2 = pz_preDcpMeas->pz_meas + u_preIndex2;
      if (fabs(pz_preMeas1->d_carrierPhase) < 1.0e-6
        || fabs(pz_preMeas2->d_carrierPhase) < 1.0e-6)
      {
        continue;
      }
      d_wave1 = wavelength((gnss_SignalType)pz_meas1->u_signal);
      d_wave2 = wavelength((gnss_SignalType)pz_meas2->u_signal);
      d_deltaGf = d_wave1 * (pz_meas1->d_carrierPhase - pz_preMeas1->d_carrierPhase)
        - d_wave2 * (pz_meas2->d_carrierPhase - pz_preMeas2->d_carrierPhase);
      if (fabs(d_deltaGf) < 0.1)
      {
        pz_dcpInfo->pu_slipTag[w_imeas1] |= DCP_NO_CYCLE_SLIP_BY_GF;
        pz_dcpInfo->pu_slipTag[w_imeas2] |= DCP_NO_CYCLE_SLIP_BY_GF;
      }
    }
  }
  return;
}

/**
 * @brief gain median of marking cycle slip by doppler using the previous epoch and current doppler observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] e_TargetSysType, target constellation
 * @param[in]  e_freqType, frequency type of satellite
 * @param[out] pf_corseRcvDrift,the result of corse receiver clock drift
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t dcp_gainMedianOfMarkCSbyDoppler(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, float* pf_corseRcvDrift)
{
  uint8_t u_status = 0;
  uint8_t u_imeas = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_obsNum = 0;
  gnss_ConstellationType u_constellation = 0;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_avgDoppler = 0.0;
  double d_wave = 0.0;
  double d_deltaTime = 0.0;
  double d_distVar = 0.0;
  double d_distVarCarrier = 0.0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  *pf_corseRcvDrift = 0.0;
  d_deltaTime = tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_preDcpMeas->z_obsTime));
  for (u_imeas = 0; u_imeas < (pz_curDcpMeas->w_measNum); ++u_imeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + u_imeas;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[e_freqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }

    if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_curFreqType != e_freqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_freqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = f_avgDoppler * d_deltaTime / d_wave;
    d_distVarCarrier = (pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase);
    pf_diff[u_obsNum++] = (float)(d_distVarCarrier - d_distVar);
  }
  u_status = gnss_ascSortMedianFloat(pf_diff, (uint32_t)u_obsNum, pf_corseRcvDrift);
  return u_status;
}

/**
 * @brief mark the cycle slip using the previous epoch and current doppler observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] e_TargetSysType, target constellation
 * @param[in] e_freqType, frequency type of satellite
 * @param[in] f_corseRcvDrift,the result of corse receiver clock drift
 * @param[out] pz_dcpInfo, store the result of detecting cycle slip
 * @return uint8_t,represent the number of normal observations
 */
uint8_t dcp_corseMarkCycleSlipByDoppler(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, float f_corseRcvDrift, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_imeas = 0;
  uint8_t u_normNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_wave = 0.0;
  float f_avgDoppler = 0.0;
  double d_distVar = 0.0;
  double d_distVarCarrier = 0.0;
  for (u_imeas = 0; u_imeas < (pz_curDcpMeas->w_measNum); ++u_imeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + u_imeas;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[e_freqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_curFreqType != e_freqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_freqType, &u_preIndex))
    {
      continue;
    }
    if ((pz_meas->u_LLI & 0x1) == 0x1 && pz_meas->f_cn0 < 42)
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = (double)f_avgDoppler * (pz_dcpInfo->f_deltaTime) / d_wave;
    d_distVarCarrier = (pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase);
    if (fabs(d_distVarCarrier - d_distVar - f_corseRcvDrift) < 1.0)
    {
      pz_dcpInfo->pu_slipTag[u_imeas] |= DCP_NO_CYCLE_SLIP_BY_DOPPLER;
      ++u_normNum;
    }
  }
  return u_normNum;
}

/**
 * @brief Log cycle slip info in DCP
 * @param[in] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] pz_dcpInfo calculated information used by TDCP or TDPR algorithm
 */
void docp_logSlip(const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t i = 0;
  char sid[4] = { 0 };
  char buff[BUFF_SIZE] = { 0 };
  char* p = buff;
  LOGI(TAG_DCP, "Dcp slip %d\n", pz_curDcpMeas->z_obsTime.q_towMsec);

  if (log_GetLogLevel() < LOG_LEVEL_I)
  {
    return;
  }
  /* sat id  */
  for (i = 0; i < pz_curDcpMeas->w_measNum; ++i)
  {
    if (gnss_cvt_Sig2FreqType(pz_curDcpMeas->pz_meas[i].u_signal) != C_GNSS_FREQ_TYPE_L1)
    {
      continue;
    }
    svid_SatString(pz_curDcpMeas->pz_meas[i].u_svid, pz_curDcpMeas->pz_meas[i].u_constellation, sid);
    p += snprintf(p, 5, "%-4s", sid);
    if ((p - buff) >= BUFF_SIZE - 5)
    {
      return;
    }
  }
  LOGI(TAG_DCP, "Sat  %s\n", buff);

  /* slip */
  memset(buff, 0, BUFF_SIZE);
  p = &buff[0];
  for (i = 0; i < pz_curDcpMeas->w_measNum; ++i)
  {
    if (gnss_cvt_Sig2FreqType(pz_curDcpMeas->pz_meas[i].u_signal) != C_GNSS_FREQ_TYPE_L1)
    {
      continue;
    }
    p += snprintf(p, 5, "%-4d", pz_dcpInfo->pu_slipTag[i]);
    if ((p - buff) >= BUFF_SIZE - 5) return;
  }
  LOGI(TAG_DCP, "Slip %s\n", buff);

  /* LLI */
  memset(buff, 0, BUFF_SIZE);
  p = &buff[0];
  for (i = 0; i < pz_curDcpMeas->w_measNum; ++i)
  {
    if (gnss_cvt_Sig2FreqType(pz_curDcpMeas->pz_meas[i].u_signal) != C_GNSS_FREQ_TYPE_L1)
    {
      continue;
    }
    p += snprintf(p, 5, "%-4d", pz_curDcpMeas->pz_meas[i].u_LLI);
    if ((p - buff) >= BUFF_SIZE - 5) return;
  }
  LOGI(TAG_DCP, "LLI  %s\n", buff);
}
/**
 * @brief doppler corse detect cycle slip using the previous epoch and current observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] e_TargetSysType, target constellation
 * @param[in] e_freqType, target frequency
 * @param[out] pz_dcpInfo, store the result of detecting cycle slip
 * @return uint8_t,represent the number of normal observations
 */
uint8_t dcp_dopplerCorseDetectSlip(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_normNum = 0;
  float f_corseRcvDrift = 0.0;
  if (dcp_gainMedianOfMarkCSbyDoppler(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, e_TargetSysType, e_freqType, &f_corseRcvDrift))
  {
    u_normNum = dcp_corseMarkCycleSlipByDoppler(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, e_TargetSysType, e_freqType, f_corseRcvDrift, pz_dcpInfo);
  }
  return u_normNum;
}
/**
 * @brief corse detect cycle slip using the previous epoch and current observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[out] pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @return void
 */
void dcp_corseDetectSlip(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  /* init */
  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pu_slipTag[u_i] = DCP_NON_DETECT;
  }

  /* slip: GF and Doppler   */
  dcp_gfCorseDetectSlip(C_GNSS_FREQ_TYPE_L1, C_GNSS_FREQ_TYPE_L2, pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
  dcp_gfCorseDetectSlip(C_GNSS_FREQ_TYPE_L1, C_GNSS_FREQ_TYPE_L5, pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
  dcp_gfCorseDetectSlip(C_GNSS_FREQ_TYPE_L2, C_GNSS_FREQ_TYPE_L5, pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[u_j], u_i))
      {
        continue;
      }
      dcp_dopplerCorseDetectSlip(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, (gnss_ConstellationType)u_i, (gnss_FreqType)u_j, pz_dcpInfo);
    }
  }
  /* log slip and LLI */
  docp_logSlip(pz_curDcpMeas, pz_dcpInfo);

  return;
}
/**
 * @brief select the optimal frequecy type for carrier phase observations
 * @param[in] pz_dcpConfig  the configuration for DCP model
 * @param[in] pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the optimal frequecy type for carrier phase observations
 */
gnss_FreqType dcp_selectCarrierOptimalFreqType(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  const dcp_interRecordInfo_t* pz_dcpInfo)
{
  gnss_FreqType z_optFreqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t u_i = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_maxNum = 0;
  uint8_t pu_validCarrierNumEachFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  double d_wave = 0.0;
  float f_ele = 0.0f;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_validCarrierNumEachFreq[u_i] = 0;
  }
  //count the vaild carrier phase for each frequency
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (0x2 == ((pz_meas->u_LLI) & 0x2))
    {
      continue;
    }
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation))
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    ++pu_validCarrierNumEachFreq[e_curFreqType];
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_validCarrierNumEachFreq[u_i] > u_maxNum)
    {
      u_maxNum = pu_validCarrierNumEachFreq[u_i];
      z_optFreqType = (gnss_FreqType)u_i;
    }
  }
  return z_optFreqType;
}

/**
 * @brief using carrier phase observations to gain the receiver clock drift
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   e_TargetSysType, target constellation
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the number of satellite for using carrier phase observations to gain the receiver clock drift
 */
uint8_t dcp_usingCarrierObsGainRcvClkDrift(const gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_ConstellationType e_TargetSysType, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_normNum = 0;
  uint8_t u_num = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_flag = 0;
  uint8_t u_i = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t pu_obsTagIndex[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_wave = 0.0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_mean = 0.0;
  float f_rejectThres = 0.0;
  float f_rejectK2 = 8.0;
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    pf_diff[u_num] = (float)((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    //the correction of earth rotation and satellite clock between epoch difference,there has been ignored
    pf_diff[u_num] -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    pu_obsTagIndex[u_num] = (uint8_t)w_iMeas;
    ++u_num;
  }
  u_flag = gnss_MadFloat(pf_diff, (uint32_t)u_num, pf_diffAbs, &f_mean);
  if (1 == u_flag)
  {
    f_rejectThres = f_rejectK2 * (f_mean / 0.6745f);
    u_normNum = 0;
    for (u_i = 0; u_i < u_num; ++u_i)
    {
      if (pf_diffAbs[u_i] > f_rejectThres && pf_diffAbs[u_i] > 10.0)
      {
        pz_dcpInfo->pu_slipTag[pu_obsTagIndex[u_i]] = DCP_NON_DETECT;
        continue;
      }
      pf_normDiff[u_normNum++] = pf_diff[u_i];
    }
    u_flag = gnss_ascSortMedianFloat(pf_normDiff, (uint32_t)u_normNum, &f_mean);
  }
  if (!u_flag)
  {
    u_normNum = 0;
  }
  else
  {
    pz_dcpInfo->pf_rcvClkDrift[e_TargetSysType] = f_mean;
  }
  return u_normNum;
}
/**
 * @brief check and deweight the carrier observations according to the residual of kalman filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_checkAndDeweightCarrierByRes(const gnss_DcpConfigOpt_t* pz_dcpConfig, float f_priorSigma0,
  const gnss_TdcpMeasBlock_t* pz_preDcpMeas, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint8_t u_satValidNum = 0;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  int16_t w_maxResIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_wave = 0.0;
  float f_ele = 0.0f;
  float f_sinEle = 0.0f;
  float f_weight = 0.0f;
  float f_res = 0.0f;
  float f_maxRes = 0.0f;
  pz_dcpInfo->f_sigma0 = 0.0f;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_dcpInfo->u_goodQualityObsNum[u_i] = 0;
  }
  //check and deweight carrier phase according to the residual of kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (0x2 == ((pz_meas->u_LLI) & 0x2))
    {
      continue;
    }
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf((float)(f_ele * DEG2RAD));
    if (f_ele > 30.0)
    {
      f_weight = 1.0f;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    f_res = (float)((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_res -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);

    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + u_constellation]);
    if (fabs(f_res) > f_maxRes && (pz_dcpInfo->pf_wP[w_iMeas]) >= 1.0)
    {
      f_maxRes = (float)fabs(f_res);
      w_maxResIndex = (int16_t)w_iMeas;
    }
    pz_dcpInfo->f_sigma0 += (f_res * (f_weight * pz_dcpInfo->pf_wP[w_iMeas]) * f_res);
    ++u_satValidNum;
    if (fabs(f_res) <= f_priorSigma0)
    {
      ++(pz_dcpInfo->u_goodQualityObsNum[u_constellation]);
    }
  }

  if (w_maxResIndex >= 0)
  {
    pz_dcpInfo->pf_wP[w_maxResIndex] = dcp_getWp(1.0f, 8.0f, f_priorSigma0, f_maxRes);
  }

  if (u_satValidNum > (pz_dcpInfo->u_paraNum))
  {
    pz_dcpInfo->f_sigma0 = sqrtf(pz_dcpInfo->f_sigma0 / (u_satValidNum - (pz_dcpInfo->u_paraNum)));
  }

  if (u_satValidNum < (pz_dcpInfo->u_paraNum) + 2)
  {
    u_status = 0;
  }
  return u_status;
}

/**
 * @brief the routeline of feed up the carrier phase to TDCP filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the number of enter TDCP filter
 */
uint8_t dcp_feedupObsTdcpFilterFloat(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t pu_obsNumPerSys[C_GNSS_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_wave = 0.0;
  float f_ele = 0.0f;
  float f_sinEle = 0.0f;
  float f_weight = 0.0f;
  float f_omc = 0.0f;
  SeqKalmanFloatVar_t z_seqKalmanFloatVar = { 0 };
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pu_obsNumPerSys[u_i] = 0;
  }
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_X[u_i] = 0.0f;
    pz_dcpInfo->pf_deltaX[u_i] = 0.0f;
  }
  for (u_i = 0; u_i < TD_MAX_Q_ELE; ++u_i)
  {
    pz_dcpInfo->pf_Q[u_i] = 0.0f;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e6f;
  }
  for (u_i = 3; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e4f;
  }
  if (0 == dcp_initSeqKalmanFloatVar(TD_MAX_PARA, &z_seqKalmanFloatVar))
  {
    return 0;
  }
  //carrier phase enter into kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (0x2 == ((pz_meas->u_LLI) & 0x2))
    {
      continue;
    }
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf(f_ele * (float)DEG2RAD);
    if (f_ele > 30.0)
    {
      f_weight = 1.0f;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    hpp_seqCleanHfloat(&z_seqKalmanFloatVar);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      hpp_seqAddHfloat((uint16_t)u_i, (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]), &z_seqKalmanFloatVar);
    }
    hpp_seqAddHfloat(3 + u_constellation, 1.0f, &z_seqKalmanFloatVar);
    //the f_omc had ignore the correction of earth rotion,because it very small for the epoch difference observation
    f_omc = (float)((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    f_omc -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_omc += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_omc -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);
    hpp_seqSetOmcFloat((float)f_omc, (float)(1.0 / ((double)f_weight * pz_dcpInfo->pf_wP[w_iMeas] + 1.0e-12)), &z_seqKalmanFloatVar);
    hpp_seqPredictStepFloat(pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar);
    hpp_seqMeasUpdateFloat(pz_dcpInfo->pf_X, pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar, 0.0f);
    ++pu_obsNumPerSys[u_constellation];
    ++u_enterFilterNum;
  }
  dcp_deinitSeqKalmanFloatVar(&z_seqKalmanFloatVar);
  pz_dcpInfo->u_paraNum = 3;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (pu_obsNumPerSys[u_i] > 0)
    {
      ++(pz_dcpInfo->u_paraNum);
    }
  }
  return u_enterFilterNum;
}
/**
 * @brief the routeline of using carrier phase to solute TDCP position
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pd_siteCoor,  the coordinate of site in ECEF
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]  pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the status of soluting TDCP position
 */
uint8_t dcp_soluteTdcpPos(const gnss_DcpConfigOpt_t* pz_dcpConfig, const double pd_siteCoor[3], const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_filterStatus = 1;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_normNum = 0;
  uint8_t u_maxIter = 10;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_isContinued = 0;
  uint8_t u_paraNum = 0;
  uint8_t u_perSysGoodQualityMaxNum = 0;
  uint8_t u_goodQualitySumNum = 0;
  float f_distDiff = 0.0f;
  float pf_preDeltaX[3] = { 0.0f };
  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pf_wP[u_i] = 1.0f;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], u_i))
    {
      continue;
    }
    pz_dcpInfo->u_enterFilterNum[u_i] = dcp_usingCarrierObsGainRcvClkDrift(pz_dcpConfig, (gnss_ConstellationType)u_i, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    u_normNum += (pz_dcpInfo->u_enterFilterNum[u_i]);
  }
  if (u_normNum <= 0)
  {
    return 0;
  }
  for (u_i = 0; u_i < u_maxIter; ++u_i)
  {
    u_enterFilterNum = dcp_feedupObsTdcpFilterFloat(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (u_enterFilterNum < (pz_dcpInfo->u_paraNum + 2))
    {
      u_filterStatus = 0;
      break;//failure case
    }
    if (0 == dcp_checkAndDeweightCarrierByRes(pz_dcpConfig, DCP_CARRIER_PRIOR_SIGMA0, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo))
    {
      u_filterStatus = 0;
      break;//failure case
    }
    u_isContinued = 0;
    for (u_j = 0; u_j < MAX_GNSS_TRK_MEAS_NUMBER; ++u_j)
    {
      if (pz_dcpInfo->pf_wP[u_j] < 1.0)
      {
        u_isContinued = 1;
        break;
      }
    }
    if (0 == u_isContinued)
    {
      break;//success case
    }

    f_distDiff = 0.0f;
    for (u_j = 0; u_j < 3; ++u_j)
    {
      f_distDiff += (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]) * (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]);
      pf_preDeltaX[u_j] = pz_dcpInfo->pf_deltaX[u_j];
    }
    if (f_distDiff < 1.0e-4)
    {
      break;//success case
    }
  }
  for (u_j = 0; u_j < 3; ++u_j)
  {
    pz_dcpInfo->pd_curXyz[u_j] = pd_siteCoor[u_j] + pz_dcpInfo->pf_deltaX[u_j];
  }

  //pz_curDcpMeas->z_posSol.z_SvStatus.u_SvInUseCount = u_enterFilterNum;
  for (u_k = C_GNSS_GPS; u_k < C_GNSS_MAX; ++u_k)
  {
    if ((pz_dcpInfo->u_goodQualityObsNum[u_k]) > u_perSysGoodQualityMaxNum)
    {
      u_perSysGoodQualityMaxNum = (pz_dcpInfo->u_goodQualityObsNum[u_k]);
    }
    u_goodQualitySumNum += (pz_dcpInfo->u_goodQualityObsNum[u_k]);
  }
  u_paraNum = pz_dcpInfo->u_paraNum;
  if (u_normNum < u_paraNum + 2 || (pz_dcpInfo->f_sigma0) > 0.2 || u_i == u_maxIter ||
    u_goodQualitySumNum < u_paraNum || (u_perSysGoodQualityMaxNum) < (3 + 1 + 1))
  {
    u_filterStatus = 0;
  }
  return u_filterStatus;
}

/**
 * @brief there will iterate delete constellion to enable the position successfully for the TDCP routeline
 * @param[in/out] pz_dcpConfig  the configuration for DCP model
 * @param[in]     pd_siteCoor,  the coordinate of site in ECEF
 * @param[in]     pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]    pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]    pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the status of soluting TDCP position
 */
uint8_t dcp_iterateDeleteSysTdcpSolute(gnss_DcpConfigOpt_t* pz_dcpConfig, const double pd_siteCoor[3], const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_filterStatus = 0;
  uint8_t u_deletedSysNum = 0;
  algo_useConstellation z_usedSys = pz_dcpConfig->z_usedSys;
  gnss_DcpConfigOpt_t z_dcpOpt = { 0 };
  gnss_FreqType z_optFreqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_deletedSys = ALGO_NON_SYS;
  memcpy(&z_dcpOpt, pz_dcpConfig, sizeof(gnss_DcpConfigOpt_t));
  do
  {
    dcp_corseDetectSlip(&z_dcpOpt, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    z_optFreqType = dcp_selectCarrierOptimalFreqType(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (z_optFreqType >= C_GNSS_FREQ_TYPE_MAX)
    {
      z_optFreqType = C_GNSS_FREQ_TYPE_L1;
    }
    z_dcpOpt.z_optimalFreqType = z_optFreqType;
    pz_dcpConfig->z_optimalFreqType = z_optFreqType;
    u_filterStatus = dcp_soluteTdcpPos(&z_dcpOpt, pd_siteCoor, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (1 == u_filterStatus)
    {
      pz_dcpInfo->u_posObainType = GNSS_DCP_POS_CARRIER;
      break;
    }
    z_deletedSys = dcp_removeSysDeleteByIndex(u_deletedSysNum);
    if (ALGO_NON_SYS == z_deletedSys)
    {
      break;
    }
    z_dcpOpt.z_usedSys = z_usedSys;
    z_dcpOpt.z_usedSys = (z_dcpOpt.z_usedSys & (~z_deletedSys));
    ++u_deletedSysNum;
  } while (u_deletedSysNum < 4);
  return u_filterStatus;
}

/**
 * @brief gain median of receiver clock drift using the previous epoch and current doppler observations and pseudo-range observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]  e_TargetSysType, target constellation
 * @param[in]  e_freqType, frequency type of satellite
 * @param[out] pf_corseRcvDrift,the result of corse receiver clock drift
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t dcp_gainMedianRcvClkDriftForPrDr(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, float* pf_corseRcvDrift)
{
  uint8_t u_status = 0;
  uint8_t u_imeas = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_obsNum = 0;
  gnss_ConstellationType u_constellation = 0;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_avgDoppler = 0.0;
  double d_deltaTime = 0.0;
  double d_distVar = 0.0;
  double d_distVarPseudo = 0.0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  *pf_corseRcvDrift = 0.0;
  d_deltaTime = tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_preDcpMeas->z_obsTime));
  for (u_imeas = 0; u_imeas < (pz_curDcpMeas->w_measNum); ++u_imeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + u_imeas;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[e_freqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }

    if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_curFreqType != e_freqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_freqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = f_avgDoppler * d_deltaTime;
    d_distVarPseudo = (pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    pf_diff[u_obsNum++] = (float)(d_distVarPseudo - d_distVar);
  }
  u_status = gnss_ascSortMedianFloat(pf_diff, (uint32_t)u_obsNum, pf_corseRcvDrift);
  return u_status;
}

/**
 * @brief pseudo-range and doppler check routline  using the previous epoch and current doppler observations and pseudo-range observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]  e_TargetSysType, target constellation
 * @param[in]  e_freqType, frequency type of satellite
 * @param[in]  f_corseRcvDrift,the result of corse receiver clock drift
 * @param[out] pz_dcpInfo, store the result of detecting cycle slip
 * @return uint8_t,represent the number of normal observations
 */
uint8_t dcp_oneFreqPrDrCheck(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, float f_corseRcvDrift, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_imeas = 0;
  uint8_t u_normNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_avgDoppler = 0.0;
  double d_distVar = 0.0;
  double d_distVarPseudo = 0.0;
  for (u_imeas = 0; u_imeas < (pz_curDcpMeas->w_measNum); ++u_imeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + u_imeas;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[e_freqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_curFreqType != e_freqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_freqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = (double)f_avgDoppler * (pz_dcpInfo->f_deltaTime);
    d_distVarPseudo = (pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    if (fabs(d_distVarPseudo - d_distVar - f_corseRcvDrift) < 10.0)
    {
      pz_dcpInfo->pu_slipTag[u_imeas] |= DCP_PR_DR_CONSISTENCY;
      ++u_normNum;
    }
  }
  return u_normNum;
}

/**
 * @brief the doppler and pseudo-range consistency check using the previous epoch and current observations for target frequency
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] e_TargetSysType, target constellation
 * @param[in] e_freqType, target frequency
 * @param[out] pz_dcpInfo, store the result of detecting cycle slip
 * @return uint8_t,represent the number of normal observations
 */
uint8_t dcp_oneFreqDopplerAndPseudoCheck(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  gnss_ConstellationType e_TargetSysType, gnss_FreqType e_freqType, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_normNum = 0;
  float f_corseRcvDrift = 0.0;
  if (dcp_gainMedianRcvClkDriftForPrDr(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, e_TargetSysType, e_freqType, &f_corseRcvDrift))
  {
    u_normNum = dcp_oneFreqPrDrCheck(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, e_TargetSysType, e_freqType, f_corseRcvDrift, pz_dcpInfo);
  }
  return u_normNum;
}

/**
 * @brief the doppler and pseudo-range consistency check using the previous epoch and current observations
 * @param[in]  pz_dcpConfig  the configuration for DCP model
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[out] pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @return void
 */
uint8_t dcp_dopplerAndPseudoConsistencyCheck(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_normNum = 0;
  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pu_slipTag[u_i] = DCP_NON_DETECT;
  }
  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[u_j], u_i))
      {
        continue;
      }
      u_normNum += dcp_oneFreqDopplerAndPseudoCheck(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, (gnss_ConstellationType)u_i, (gnss_FreqType)u_j, pz_dcpInfo);
    }
  }
  /* log with slip */
  LOGI(TAG_DCP, "Doppler pseudo consistency check\n");
  docp_logSlip(pz_curDcpMeas, pz_dcpInfo);

  return u_normNum;
}

/**
 * @brief select the optimal frequecy type for doppler observations
 * @param[in] pz_dcpConfig  the configuration for DCP model
 * @param[in] pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the optimal frequecy type for doppler observations
 */
gnss_FreqType dcp_selectDopplerOptimalFreqType(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  const dcp_interRecordInfo_t* pz_dcpInfo)
{
  gnss_FreqType z_optFreqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t u_i = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_maxNum = 0;
  uint8_t pu_validDopplerNumEachFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  float f_ele = 0.0f;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_validDopplerNumEachFreq[u_i] = 0;
  }
  //count the vaild doppler for each frequency
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation))
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-6 && fabs(pz_preMeas->d_doppler) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    ++pu_validDopplerNumEachFreq[e_curFreqType];
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_validDopplerNumEachFreq[u_i] > u_maxNum)
    {
      u_maxNum = pu_validDopplerNumEachFreq[u_i];
      z_optFreqType = (gnss_FreqType)u_i;
    }
  }
  return z_optFreqType;
}

/**
 * @brief select the optimal frequecy type for pseudo-range observations
 * @param[in] pz_dcpConfig  the configuration for DCP model
 * @param[in] pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the optimal frequecy type for pseudo-range observations
 */
gnss_FreqType dcp_selectPseudoOptimalFreqType(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  const dcp_interRecordInfo_t* pz_dcpInfo)
{
  gnss_FreqType z_optFreqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t u_i = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_maxNum = 0;
  uint8_t pu_validPseudoNumEachFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  float f_ele = 0.0f;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_validPseudoNumEachFreq[u_i] = 0;
  }
  //count the vaild pseudo-range for each frequency
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation))
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    ++pu_validPseudoNumEachFreq[e_curFreqType];
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_validPseudoNumEachFreq[u_i] > u_maxNum)
    {
      u_maxNum = pu_validPseudoNumEachFreq[u_i];
      z_optFreqType = (gnss_FreqType)u_i;
    }
  }
  return z_optFreqType;
}

/**
 * @brief using doppler observations to gain the receiver clock drift
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   e_TargetSysType, target constellation
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDDR algorithm
 * @return the number of satellite for using doppler observations to gain the receiver clock drift
 */
uint8_t dcp_usingDopplerObsGainRcvClkDrift(const gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_ConstellationType e_TargetSysType, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_normNum = 0;
  uint8_t u_num = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_flag = 0;
  uint8_t u_i = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t pu_obsTagIndex[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_avgDoppler = 0.0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_mean = 0.0;
  float f_rejectThres = 0.0;
  float f_rejectK2 = 8.0;
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-6 && fabs(pz_preMeas->d_doppler) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    pf_diff[u_num] = (float)(f_avgDoppler * (pz_dcpInfo->f_deltaTime));
    //the correction of earth rotation and satellite clock between epoch difference,there has been ignored
    pf_diff[u_num] -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    pu_obsTagIndex[u_num] = (uint8_t)w_iMeas;
    ++u_num;
  }
  u_flag = gnss_MadFloat(pf_diff, (uint32_t)u_num, pf_diffAbs, &f_mean);
  if (1 == u_flag)
  {
    f_rejectThres = f_rejectK2 * (f_mean / 0.6745f);
    u_normNum = 0;
    for (u_i = 0; u_i < u_num; ++u_i)
    {
      if (pf_diffAbs[u_i] > f_rejectThres && pf_diffAbs[u_i] > 10.0)
      {
        pz_dcpInfo->pu_slipTag[pu_obsTagIndex[u_i]] = DCP_NON_DETECT;
        continue;
      }
      pf_normDiff[u_normNum++] = pf_diff[u_i];
    }
    u_flag = gnss_ascSortMedianFloat(pf_normDiff, (uint32_t)u_normNum, &f_mean);
  }
  if (!u_flag)
  {
    u_normNum = 0;
  }
  else
  {
    pz_dcpInfo->pf_rcvClkDrift[e_TargetSysType] = f_mean;
  }
  return u_normNum;
}

/**
 * @brief the routeline of feed up the doppler to TDDR filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the number of enter TDDR filter
 */
uint8_t dcp_feedupObsTddrFilterFloat(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint8_t pu_obsNumPerSys[C_GNSS_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_avgDoppler = 0.0;
  float f_ele = 0.0;
  float f_sinEle = 0.0;
  float f_weight = 0.0;
  float f_omc = 0.0;
  SeqKalmanFloatVar_t z_seqKalmanFloatVar = { 0 };
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pu_obsNumPerSys[u_i] = 0;
  }
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_X[u_i] = 0.0;
    pz_dcpInfo->pf_deltaX[u_i] = 0.0;
  }
  for (u_i = 0; u_i < TD_MAX_Q_ELE; ++u_i)
  {
    pz_dcpInfo->pf_Q[u_i] = 0.0;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e6;
  }
  for (u_i = 3; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e4;
  }
  if (0 == dcp_initSeqKalmanFloatVar(TD_MAX_PARA, &z_seqKalmanFloatVar))
  {
    return 0;
  }
  //carrier phase enter into kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-6 && fabs(pz_preMeas->d_doppler) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf((float)(f_ele * DEG2RAD));
    if (f_ele > 30.0)
    {
      f_weight = 1.0;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    hpp_seqCleanHfloat(&z_seqKalmanFloatVar);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      hpp_seqAddHfloat((uint16_t)u_i, (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]), &z_seqKalmanFloatVar);
    }
    hpp_seqAddHfloat(3 + u_constellation, 1.0, &z_seqKalmanFloatVar);
    //the f_omc had ignore the correction of earth rotion,because it very small for the epoch difference observation
    f_omc = (float)(f_avgDoppler * (pz_dcpInfo->f_deltaTime));
    f_omc -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_omc += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_omc -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);
    hpp_seqSetOmcFloat((float)f_omc, (float)(1.0 / ((double)f_weight * pz_dcpInfo->pf_wP[w_iMeas] + 1.0e-12)), &z_seqKalmanFloatVar);
    hpp_seqPredictStepFloat(pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar);
    hpp_seqMeasUpdateFloat(pz_dcpInfo->pf_X, pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar, 0.0);
    ++pu_obsNumPerSys[u_constellation];
    ++u_enterFilterNum;
  }
  dcp_deinitSeqKalmanFloatVar(&z_seqKalmanFloatVar);
  pz_dcpInfo->u_paraNum = 3;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (pu_obsNumPerSys[u_i] > 0)
    {
      ++(pz_dcpInfo->u_paraNum);
    }
  }
  return u_enterFilterNum;
}

/**
 * @brief check and deweight the doppler observations according to the residual of kalman filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_checkAndDeweightDopplerByRes(const gnss_DcpConfigOpt_t* pz_dcpConfig, float f_priorSigma0,
  const gnss_TdcpMeasBlock_t* pz_preDcpMeas, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint8_t u_satValidNum = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  int16_t w_maxResIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_avgDoppler = 0.0;
  float f_ele = 0.0;
  float f_sinEle = 0.0;
  float f_weight = 0.0;
  float f_res = 0.0;
  float f_maxRes = 0.0;
  pz_dcpInfo->f_sigma0 = 0.0;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_dcpInfo->u_goodQualityObsNum[u_i] = 0;
  }
  //check and deweight carrier phase according to the residual of kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-6 && fabs(pz_preMeas->d_doppler) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf((float)(f_ele * DEG2RAD));
    if (f_ele > 30.0)
    {
      f_weight = 1.0;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    f_res = (float)(f_avgDoppler * (pz_dcpInfo->f_deltaTime));
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_res -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);

    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + u_constellation]);
    if (fabs(f_res) > f_maxRes && (pz_dcpInfo->pf_wP[w_iMeas]) >= 1.0)
    {
      f_maxRes = (float)fabs(f_res);
      w_maxResIndex = (int16_t)w_iMeas;
    }
    pz_dcpInfo->f_sigma0 += (f_res * (f_weight * pz_dcpInfo->pf_wP[w_iMeas]) * f_res);
    ++u_satValidNum;
    if (fabs(f_res) <= f_priorSigma0)
    {
      ++(pz_dcpInfo->u_goodQualityObsNum[u_constellation]);
    }
  }

  if (w_maxResIndex >= 0)
  {
    pz_dcpInfo->pf_wP[w_maxResIndex] = dcp_getWp(1.0, 8.0, f_priorSigma0, f_maxRes);
  }

  if (u_satValidNum > (pz_dcpInfo->u_paraNum))
  {
    pz_dcpInfo->f_sigma0 = sqrtf(pz_dcpInfo->f_sigma0 / (u_satValidNum - (pz_dcpInfo->u_paraNum)));
  }

  if (u_satValidNum < (pz_dcpInfo->u_paraNum) + 2)
  {
    u_status = 0;
  }
  return u_status;
}

/**
 * @brief the routeline of using doppler to solute TDDR position
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pd_siteCoor,  the coordinate of site in ECEF
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]  pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDDR algorithm
 * @return the status of soluting TDDR position
 */
uint8_t dcp_soluteTddrPos(const gnss_DcpConfigOpt_t* pz_dcpConfig, const double pd_siteCoor[3], const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_filterStatus = 1;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_normNum = 0;
  uint8_t u_maxIter = 10;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_isContinued = 0;
  uint8_t u_paraNum = 0;
  uint8_t u_perSysGoodQualityMaxNum = 0;
  uint8_t u_goodQualitySumNum = 0;
  float f_distDiff = 0.0;
  float pf_preDeltaX[3] = { 0.0 };
  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pf_wP[u_i] = 1.0;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], u_i))
    {
      continue;
    }
    pz_dcpInfo->u_enterFilterNum[u_i] = dcp_usingDopplerObsGainRcvClkDrift(pz_dcpConfig, (gnss_ConstellationType)u_i, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    u_normNum += (pz_dcpInfo->u_enterFilterNum[u_i]);
  }
  if (u_normNum <= 0)
  {
    return 0;
  }
  for (u_i = 0; u_i < u_maxIter; ++u_i)
  {
    u_enterFilterNum = dcp_feedupObsTddrFilterFloat(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (u_enterFilterNum < (pz_dcpInfo->u_paraNum + 2))
    {
      u_filterStatus = 0;
      break;//failure case
    }
    if (0 == dcp_checkAndDeweightDopplerByRes(pz_dcpConfig, DCP_DOPPLE_PRIOR_SIGMA0, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo))
    {
      u_filterStatus = 0;
      break;//failure case
    }
    u_isContinued = 0;
    for (u_j = 0; u_j < MAX_GNSS_TRK_MEAS_NUMBER; ++u_j)
    {
      if (pz_dcpInfo->pf_wP[u_j] < 1.0)
      {
        u_isContinued = 1;
        break;
      }
    }
    if (0 == u_isContinued)
    {
      break;//success case
    }

    f_distDiff = 0.0;
    for (u_j = 0; u_j < 3; ++u_j)
    {
      f_distDiff += (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]) * (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]);
      pf_preDeltaX[u_j] = pz_dcpInfo->pf_deltaX[u_j];
    }
    if (f_distDiff < 1.0e-4)
    {
      break;//success case
    }
  }
  for (u_j = 0; u_j < 3; ++u_j)
  {
    pz_dcpInfo->pd_curXyz[u_j] = pd_siteCoor[u_j] + pz_dcpInfo->pf_deltaX[u_j];
  }

  //pz_curDcpMeas->z_posSol.z_SvStatus.u_SvInUseCount = u_enterFilterNum;

  for (u_k = C_GNSS_GPS; u_k < C_GNSS_MAX; ++u_k)
  {
    if ((pz_dcpInfo->u_goodQualityObsNum[u_k]) > u_perSysGoodQualityMaxNum)
    {
      u_perSysGoodQualityMaxNum = (pz_dcpInfo->u_goodQualityObsNum[u_k]);
    }
    u_goodQualitySumNum += (pz_dcpInfo->u_goodQualityObsNum[u_k]);
  }
  u_paraNum = pz_dcpInfo->u_paraNum;
  if (u_normNum < u_paraNum + 2 || (pz_dcpInfo->f_sigma0) > 3.0 || u_i == u_maxIter ||
    u_goodQualitySumNum < u_paraNum || (u_perSysGoodQualityMaxNum) < (3 + 1 + 1))
  {
    u_filterStatus = 0;
  }
  return u_filterStatus;
}

/**
 * @brief using pseudo-range observations to gain the receiver clock drift
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   e_TargetSysType, target constellation
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDDR algorithm
 * @return the number of satellite for using pseudo-range observations to gain the receiver clock drift
 */
uint8_t dcp_usingPseudoObsGainRcvClkDrift(const gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_ConstellationType e_TargetSysType, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_normNum = 0;
  uint8_t u_num = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_flag = 0;
  uint8_t u_i = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t pu_obsTagIndex[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0f };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0f };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0f };
  float f_mean = 0.0f;
  float f_rejectThres = 0.0f;
  float f_rejectK2 = 8.0f;
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    pf_diff[u_num] = (float)(pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    //the correction of earth rotation and satellite clock between epoch difference,there has been ignored
    pf_diff[u_num] -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    pu_obsTagIndex[u_num] = (uint8_t)w_iMeas;
    ++u_num;
  }
  u_flag = gnss_MadFloat(pf_diff, (uint32_t)u_num, pf_diffAbs, &f_mean);
  if (1 == u_flag)
  {
    f_rejectThres = (float)(f_rejectK2 * (f_mean / 0.6745));
    u_normNum = 0;
    for (u_i = 0; u_i < u_num; ++u_i)
    {
      if (pf_diffAbs[u_i] > f_rejectThres && pf_diffAbs[u_i] > 10.0)
      {
        pz_dcpInfo->pu_slipTag[pu_obsTagIndex[u_i]] = DCP_NON_DETECT;
        continue;
      }
      pf_normDiff[u_normNum++] = pf_diff[u_i];
    }
    u_flag = gnss_ascSortMedianFloat(pf_normDiff, (uint32_t)u_normNum, &f_mean);
  }
  if (!u_flag)
  {
    u_normNum = 0;
  }
  else
  {
    pz_dcpInfo->pf_rcvClkDrift[e_TargetSysType] = f_mean;
  }
  return u_normNum;
}

/**
 * @brief the routeline of feed up the pseudo-range observation to TDPR filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the number of enter TDPR filter
 */
uint8_t dcp_feedupObsTdprFilterFloat(const gnss_DcpConfigOpt_t* pz_dcpConfig, const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_i = 0;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint8_t pu_obsNumPerSys[C_GNSS_MAX] = { 0 };
  uint16_t w_iMeas = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_ele = 0.0;
  float f_sinEle = 0.0;
  float f_weight = 0.0;
  float f_omc = 0.0;
  SeqKalmanFloatVar_t z_seqKalmanFloatVar = { 0 };
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pu_obsNumPerSys[u_i] = 0;
  }
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_X[u_i] = 0.0;
    pz_dcpInfo->pf_deltaX[u_i] = 0.0;
  }
  for (u_i = 0; u_i < TD_MAX_Q_ELE; ++u_i)
  {
    pz_dcpInfo->pf_Q[u_i] = 0.0;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e6;
  }
  for (u_i = 3; u_i < TD_MAX_PARA; ++u_i)
  {
    pz_dcpInfo->pf_Q[IUTM(u_i, u_i)] = 1.0e4;
  }
  if (0 == dcp_initSeqKalmanFloatVar(TD_MAX_PARA, &z_seqKalmanFloatVar))
  {
    return 0;
  }
  //carrier phase enter into kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf(f_ele * (float)DEG2RAD);
    if (f_ele > 30.0)
    {
      f_weight = 1.0f;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    hpp_seqCleanHfloat(&z_seqKalmanFloatVar);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      hpp_seqAddHfloat((uint16_t)u_i, (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]), &z_seqKalmanFloatVar);
    }
    hpp_seqAddHfloat(3 + u_constellation, 1.0f, &z_seqKalmanFloatVar);
    //the f_omc had ignore the correction of earth rotion,because it very small for the epoch difference observation
    f_omc = (float)(pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    f_omc -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_omc += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_omc -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);
    LOGD(TAG_DCP, "%d,%2d,omc:%4.1f,weight:%4.1f\n", pz_meas->u_constellation, pz_meas->u_svid, f_omc, (double)f_weight * pz_dcpInfo->pf_wP[w_iMeas]);
    hpp_seqSetOmcFloat((float)f_omc, (float)(1.0 / ((double)f_weight * pz_dcpInfo->pf_wP[w_iMeas] + 1.0e-12)), &z_seqKalmanFloatVar);
    hpp_seqPredictStepFloat(pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar);
    hpp_seqMeasUpdateFloat(pz_dcpInfo->pf_X, pz_dcpInfo->pf_deltaX, pz_dcpInfo->pf_Q, &z_seqKalmanFloatVar, 0.0);
    ++pu_obsNumPerSys[u_constellation];
    ++u_enterFilterNum;
  }
  dcp_deinitSeqKalmanFloatVar(&z_seqKalmanFloatVar);
  pz_dcpInfo->u_paraNum = 3;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (pu_obsNumPerSys[u_i] > 0)
    {
      ++(pz_dcpInfo->u_paraNum);
    }
  }
  return u_enterFilterNum;
}

/**
 * @brief check and deweight the pseo-range observations according to the residual of kalman filter
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_checkAndDeweightPseudoByRes(const gnss_DcpConfigOpt_t* pz_dcpConfig, float f_priorSigma0,
  const gnss_TdcpMeasBlock_t* pz_preDcpMeas, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint8_t u_satValidNum = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  int16_t w_maxResIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_ele = 0.0f;
  float f_sinEle = 0.0f;
  float f_weight = 0.0f;
  float f_res = 0.0f;
  float f_maxRes = 0.0f;
  pz_dcpInfo->f_sigma0 = 0.0f;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_dcpInfo->u_goodQualityObsNum[u_i] = 0;
  }
  //check and deweight carrier phase according to the residual of kalman filter
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }

    f_ele = (pz_preDcpMeas->pz_satInfo[w_preSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_sinEle = sinf(f_ele * (float)DEG2RAD);
    if (f_ele > 30.0)
    {
      f_weight = 1.0f;
    }
    else
    {
      f_weight = 4.0f * f_sinEle * f_sinEle;
    }
    f_res = (float)(pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_res -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);

    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + u_constellation]);
    LOGD(TAG_DCP, "%d,%2d,res:%4.1f,weight:%4.1f\n", pz_meas->u_constellation, pz_meas->u_svid, f_res, (double)f_weight * pz_dcpInfo->pf_wP[w_iMeas]);

    if (fabsf(f_res) > f_maxRes && (pz_dcpInfo->pf_wP[w_iMeas]) >= 1.0)
    {
      f_maxRes = fabsf(f_res);
      w_maxResIndex = (int16_t)w_iMeas;
    }
    pz_dcpInfo->f_sigma0 += (f_res * (f_weight * pz_dcpInfo->pf_wP[w_iMeas]) * f_res);
    ++u_satValidNum;
    if (fabsf(f_res) <= f_priorSigma0)
    {
      ++(pz_dcpInfo->u_goodQualityObsNum[u_constellation]);
    }
  }
  /* deweight */
  if (w_maxResIndex >= 0)
  {
    double wp = pz_dcpInfo->pf_wP[w_maxResIndex];
    pz_dcpInfo->pf_wP[w_maxResIndex] = dcp_getWp(1.0f, 8.0f, f_priorSigma0, f_maxRes);
    LOGI(TAG_DCP, "deweight, %d,%2d,res:%4.1f,wp:%4.1f->%4.1f\n",
      pz_curDcpMeas->pz_meas[w_maxResIndex].u_constellation, pz_curDcpMeas->pz_meas[w_maxResIndex].u_svid,
      f_maxRes, wp, pz_dcpInfo->pf_wP[w_maxResIndex]);
  }

  if (u_satValidNum > (pz_dcpInfo->u_paraNum))
  {
    pz_dcpInfo->f_sigma0 = sqrtf(pz_dcpInfo->f_sigma0 / (float)(u_satValidNum - (pz_dcpInfo->u_paraNum)));
    LOGI(TAG_DCP, "Sigma:%.2f\n", pz_dcpInfo->f_sigma0);
  }

  if (u_satValidNum < (pz_dcpInfo->u_paraNum) + 2)
  {
    u_status = 0;
  }
  return u_status;
}

/**
 * @brief the routeline of using pseudo-range to solute TDPR position
 * @param[in]   pz_dcpConfig  the configuration for DCP model
 * @param[in]   pd_siteCoor,  the coordinate of site in ECEF
 * @param[in]   pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]  pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]  pz_dcpInfo,   the calculated information used by TDPR algorithm
 * @return the status of soluting TDPR position
 */
uint8_t dcp_soluteTdprPos(const gnss_DcpConfigOpt_t* pz_dcpConfig, const double pd_siteCoor[3], const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_filterStatus = 1;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_normNum = 0;
  uint8_t u_maxIter = 10;
  uint8_t u_enterFilterNum = 0;
  uint8_t u_isContinued = 0;
  uint8_t u_paraNum = 0;
  uint8_t u_perSysGoodQualityMaxNum = 0;
  uint8_t u_goodQualitySumNum = 0;
  float f_distDiff = 0.0f;
  float pf_preDeltaX[3] = { 0.0f };

  for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; ++u_i)
  {
    pz_dcpInfo->pf_wP[u_i] = 1.0f;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dcpConfig->pq_isSeparateBDS2And3[pz_dcpConfig->z_optimalFreqType], u_i))
    {
      continue;
    }
    pz_dcpInfo->u_enterFilterNum[u_i] = dcp_usingPseudoObsGainRcvClkDrift(pz_dcpConfig, (gnss_ConstellationType)u_i, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    u_normNum += (pz_dcpInfo->u_enterFilterNum[u_i]);
  }
  if (u_normNum <= 0)
  {
    return 0;
  }
  for (u_i = 0; u_i < u_maxIter; ++u_i)
  {
    LOGD(TAG_DCP, "Iter:%d\n", u_i);
    u_enterFilterNum = dcp_feedupObsTdprFilterFloat(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (u_enterFilterNum < (pz_dcpInfo->u_paraNum + 2))
    {
      u_filterStatus = 0;
      break; //failure case
    }
    if (0 == dcp_checkAndDeweightPseudoByRes(pz_dcpConfig, DCP_PSEUDO_PRIOR_SIGMA0, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo))
    {
      u_filterStatus = 0;
      break; //failure case
    }
    u_isContinued = 0;
    for (u_j = 0; u_j < MAX_GNSS_TRK_MEAS_NUMBER; ++u_j)
    {
      if (pz_dcpInfo->pf_wP[u_j] < 1.0)
      {
        u_isContinued = 1;
        break;
      }
    }
    if (0 == u_isContinued)
    {
      break; //success case
    }

    f_distDiff = 0.0f;
    for (u_j = 0; u_j < 3; ++u_j)
    {
      f_distDiff += (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]) * (pz_dcpInfo->pf_deltaX[u_j] - pf_preDeltaX[u_j]);
      pf_preDeltaX[u_j] = pz_dcpInfo->pf_deltaX[u_j];
    }
    if (f_distDiff < 1.0e-4)
    {
      break;//success case
    }
  }
  for (u_j = 0; u_j < 3; ++u_j)
  {
    pz_dcpInfo->pd_curXyz[u_j] = pd_siteCoor[u_j] + pz_dcpInfo->pf_deltaX[u_j];
  }
  for (u_k = C_GNSS_GPS; u_k < C_GNSS_MAX; ++u_k)
  {
    if ((pz_dcpInfo->u_goodQualityObsNum[u_k]) > u_perSysGoodQualityMaxNum)
    {
      u_perSysGoodQualityMaxNum = (pz_dcpInfo->u_goodQualityObsNum[u_k]);
    }
    u_goodQualitySumNum += (pz_dcpInfo->u_goodQualityObsNum[u_k]);
  }
  u_paraNum = pz_dcpInfo->u_paraNum;
  if (u_normNum < u_paraNum + 2 || (pz_dcpInfo->f_sigma0) > 3.0 || u_i == u_maxIter ||
    (u_goodQualitySumNum) < u_paraNum || (u_perSysGoodQualityMaxNum) < (3 + 1 + 1))
  {
    u_filterStatus = 0;
  }
  if (1 == u_filterStatus)
  {
    //pz_curDcpMeas->z_posSol.z_SvStatus.u_SvInUseCount = u_enterFilterNum;
    LOGI(TAG_DCP, "Base x,y,z: %f, %f, %f, dx,dy,dz: %.2f, %.2f, %.2f\n", pd_siteCoor[0], pd_siteCoor[1],
      pd_siteCoor[2], pz_dcpInfo->pf_deltaX[0],
      pz_dcpInfo->pf_deltaX[1], pz_dcpInfo->pf_deltaX[2]);
  }
  return u_filterStatus;
}

/**
 * @brief there will iterate delete constellion to enable the position successfully for the TDPR or TDDR routeline
 * @param[in/out] pz_dcpConfig  the configuration for DCP model
 * @param[in]     pd_siteCoor,  the coordinate of site in ECEF
 * @param[in]     pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]    pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]    pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @return the status of soluting TDPR or TDDR position
 */
uint8_t dcp_iterateDeleteSysTdprOrTddrSolute(gnss_DcpConfigOpt_t* pz_dcpConfig, const double pd_siteCoor[3], const gnss_TdcpMeasBlock_t* pz_preDcpMeas,
  gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_filterStatus = 0;
  uint8_t u_deletedSysNum = 0;
  uint8_t u_posObainType = GNSS_DCP_POS_INVALID;
  algo_useConstellation z_usedSys = pz_dcpConfig->z_usedSys;
  gnss_DcpConfigOpt_t z_dcpOpt = { 0 };
  gnss_FreqType z_optFreqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_deletedSys = ALGO_NON_SYS;
  memcpy(&z_dcpOpt, pz_dcpConfig, sizeof(gnss_DcpConfigOpt_t));
  do
  {
    dcp_dopplerAndPseudoConsistencyCheck(&z_dcpOpt, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
    if (fabsf(pz_dcpInfo->f_deltaTime) < 10.0)
    {
      z_optFreqType = dcp_selectDopplerOptimalFreqType(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
      if (z_optFreqType >= C_GNSS_FREQ_TYPE_MAX)
      {
        z_optFreqType = C_GNSS_FREQ_TYPE_L1;
      }
      z_dcpOpt.z_optimalFreqType = z_optFreqType;
      pz_dcpConfig->z_optimalFreqType = z_optFreqType;
      u_filterStatus = dcp_soluteTddrPos(&z_dcpOpt, pd_siteCoor, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
      u_posObainType = GNSS_DCP_POS_DOPPLER;
    }
    else
    {
      z_optFreqType = dcp_selectPseudoOptimalFreqType(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
      if (z_optFreqType >= C_GNSS_FREQ_TYPE_MAX)
      {
        z_optFreqType = C_GNSS_FREQ_TYPE_L1;
      }
      z_dcpOpt.z_optimalFreqType = z_optFreqType;
      pz_dcpConfig->z_optimalFreqType = z_optFreqType;
      u_filterStatus = dcp_soluteTdprPos(&z_dcpOpt, pd_siteCoor, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo);
      u_posObainType = GNSS_DCP_POS_PSEUDO;
    }
    if (1 == u_filterStatus)
    {
      pz_dcpInfo->u_posObainType = u_posObainType;
      break;
    }
    z_deletedSys = dcp_removeSysDeleteByIndex(u_deletedSysNum);
    if (ALGO_NON_SYS == z_deletedSys)
    {
      break;
    }
    z_dcpOpt.z_usedSys = z_usedSys;
    z_dcpOpt.z_usedSys = (z_dcpOpt.z_usedSys & (~z_deletedSys));
    ++u_deletedSysNum;
  } while (u_deletedSysNum < 4);
  return u_filterStatus;
}
/**
  * @brief according the signal type to determine whether BDS2 and BDS3 are separate in DCP
  * @param[in]     z_targetFreq is the target frequency
  * @param[in]     pz_curDcpMeas TDCP measure block of current epoch
  * @return TRUE represent BDS2 and BDS3 will be separated and FALSE represent BDS2 and BDS3 will not be separated
  */
BOOL dcp_isSeparateBDS2And3(gnss_FreqType z_targetFreq, const gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  BOOL q_isSeparateBDS2And3 = FALSE;
  uint16_t w_iMeas = 0;
  BOOL q_isBDS3 = FALSE;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SignalType u_BDS2SignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_BDS3SignalType = C_GNSS_SIG_MAX;
  const GnssMeas_t* pz_meas = NULL;
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (C_GNSS_BDS2 != (pz_meas->u_constellation) && C_GNSS_BDS3 != (pz_meas->u_constellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_curFreqType != z_targetFreq)
    {
      continue;
    }
    if (C_GNSS_SIG_MAX != u_BDS2SignalType && C_GNSS_SIG_MAX != u_BDS3SignalType)
    {
      break;
    }
    q_isBDS3 = gnss_isBDS3Sat(pz_meas->u_svid, pz_meas->u_constellation);
    if (TRUE == q_isBDS3)
    {
      u_BDS3SignalType = pz_meas->u_signal;
    }
    else
    {
      u_BDS2SignalType = pz_meas->u_signal;
    }
  }
  if (u_BDS2SignalType != u_BDS3SignalType)
  {
    q_isSeparateBDS2And3 = TRUE;
  }
  return q_isSeparateBDS2And3;
}
/**
 * @brief calculate the current position using the previous epoch and current observations by TDCP or TDPR
 * @param[in/out] pz_dcpConfig the configuration for the DCP model
 * @param[in]     pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]    pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]    pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_gainTDpos(gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  dcp_interRecordInfo_t* pz_dcpInfo)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  pz_dcpInfo->u_posObainType = GNSS_DCP_POS_INVALID;
  if (1 == (pz_dcpConfig->u_enableCalCurrentSatPos))
  {
    dcp_fillupCurSatInfo(pz_preDcpMeas, pz_curDcpMeas); // current satellite position and vel
  }
  dcp_updatePreSatInfo(pz_curDcpMeas, pz_preDcpMeas); // match pre and cur ephemeris`s iode
  dcp_gainSite2SatInfo(pz_preDcpMeas->z_posSol.d_xyz, pz_preDcpMeas, pz_curDcpMeas); // site to satellite unit and dis
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pz_dcpConfig->pq_isSeparateBDS2And3[u_i] = dcp_isSeparateBDS2And3(u_i, pz_curDcpMeas);
  }
  // TDCP
  if (0 == dcp_iterateDeleteSysTdcpSolute(pz_dcpConfig, pz_preDcpMeas->z_posSol.d_xyz, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo))
  {
    LOGI(TAG_DCP, "cp failed\n");
    // TDPR / TDDR
    if (1 == pz_dcpConfig->u_enableTDDRorTDPR &&
      1 == dcp_iterateDeleteSysTdprOrTddrSolute(pz_dcpConfig, pz_preDcpMeas->z_posSol.d_xyz, pz_preDcpMeas, pz_curDcpMeas, pz_dcpInfo))
    {
      if (GNSS_FIX_FLAG_FIXED == (pz_curDcpMeas->z_posSol.u_fixFlag))
      {
        pz_curDcpMeas->z_posSol.u_fixFlag = GNSS_FIX_FLAG_FLOATING;
      }
      else if (GNSS_FIX_FLAG_FLOATING == (pz_curDcpMeas->z_posSol.u_fixFlag))
      {
        pz_curDcpMeas->z_posSol.u_fixFlag = GNSS_FIX_FLAG_DGNSS;
      }
    }
  }

  // save result
  pz_curDcpMeas->z_posSol.u_DcpPosType = pz_dcpInfo->u_posObainType;
  if (GNSS_DCP_POS_INVALID != (pz_dcpInfo->u_posObainType))
  {
    u_status = 1;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_curDcpMeas->z_posSol.d_xyz[u_i] = pz_dcpInfo->pd_curXyz[u_i];
      pz_curDcpMeas->z_posSol.f_velXyz[u_i] = (pz_dcpInfo->pf_deltaX[u_i]) / (pz_dcpInfo->f_deltaTime);
    }

    LOGI(TAG_DCP, "Dcp suc %d, flag:%d->%d, type:%d\n", pz_curDcpMeas->z_obsTime.q_towMsec,
      pz_preDcpMeas->z_posSol.u_fixFlag, pz_curDcpMeas->z_posSol.u_fixFlag, pz_curDcpMeas->z_posSol.u_DcpPosType);
  }
  else
  {
    u_status = 0;
    pz_curDcpMeas->z_posSol.u_fixFlag = GNSS_FIX_FLAG_INVALID;
    LOGI(TAG_DCP, "Dcp failed %d\n", pz_curDcpMeas->z_obsTime.q_towMsec);
  }
  return u_status;
}