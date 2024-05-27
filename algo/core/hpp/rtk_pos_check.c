#include "rtk_pos_check.h"
#include "cmn_def.h"
#include "gnss_common.h"
#include "gnss_type.h"
#include "mw_log.h"
#include "rtk_type.h"
#include "mw_alloc.h"
#include "cmn_utils.h"

/**
 * @brief              init history pos structure
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @return             void
 */
void rtk_initHistoryPos(rtk_history_pos_t* pz_historyPos_t)
{
  uint8_t u_i = 0;
  pz_historyPos_t->u_num = 0;
  pz_historyPos_t->q_DiffFlagWithSPP = 0;
  tm_initGpsTime(&pz_historyPos_t->z_tor);
  for (u_i = 0; u_i < MAX_NUM_HISTORY_POS; u_i++)
  {
    pz_historyPos_t->pd_historyPos[u_i][0] = 0.0;
    pz_historyPos_t->pd_historyPos[u_i][1] = 0.0;
    pz_historyPos_t->pd_historyPos[u_i][2] = 0.0;
    pz_historyPos_t->pd_SPPvelENU[u_i][0] = 0.0;
    pz_historyPos_t->pd_SPPvelENU[u_i][1] = 0.0;
    pz_historyPos_t->pd_SPPvelENU[u_i][2] = 0.0;
    pz_historyPos_t->pd_deltaTime[u_i] = 0.0;
    pz_historyPos_t->pu_posType[u_i] = RTK_FILTER_SOL_STAT_NONE;
  }
}
/**
 * @brief              push the filter pos in history pos structure
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]          u_solTag = RTK_FILTER_POS_... the pushed filter sol type
 * @param[in]          z_tor time of pushed filter pos
 * @param[in]          u_diffFlagWithSPP flag of whether rtk sol has significant difference with spp
 * @param[in]          pz_posSPP result of spp
 * @return             TRUE for success ,FALSE for error case
 */
BOOL rtk_pushHistoryPos(rtk_filterInfo_t* pz_RTKfilterInfo,const uint16_t u_solTag,
  const GpsTime_t z_tor,const uint32_t q_diffFlagWithSPP, const gnss_PositionFix_t* pz_posSPP)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  double d_diffTime = 0.0;
  rtk_history_pos_t* pz_historyPos_t = NULL;
  rtk_filter_sol_t* pz_filterSol = NULL;
  /* threshold define begin */
  const double d_MaxDiffTime = 2.5; //max diff time
  /* threshold define end */

  pz_historyPos_t = pz_RTKfilterInfo->pz_historyPos_t;
  pz_filterSol = pz_RTKfilterInfo->pz_filterSol;
  if (pz_historyPos_t == NULL || pz_filterSol == NULL)
  {
    return FALSE;
  }
  if (gnss_Dot(pz_filterSol->pd_filterPos[u_solTag], pz_filterSol->pd_filterPos[u_solTag], 3) < FABS_ZEROS)
  {
    return FALSE;
  }
  d_diffTime = tm_GpsTimeDiff(&z_tor, &pz_historyPos_t->z_tor);
  if (pz_historyPos_t->u_num == 0 || d_diffTime > d_MaxDiffTime || d_diffTime < FABS_ZEROS)
  {
    rtk_initHistoryPos(pz_historyPos_t);
    for (u_j = 0; u_j < 3; u_j++)
    {
      pz_historyPos_t->pd_historyPos[0][u_j] = pz_filterSol->pd_filterPos[u_solTag][u_j];
    }
    pz_historyPos_t->u_num = 1;
  }
  else
  {
    for (u_i = MAX_NUM_HISTORY_POS - 1; u_i >= 1; u_i--)
    {
      pz_historyPos_t->pu_posType[u_i] = pz_historyPos_t->pu_posType[u_i - 1];
      for (u_j = 0; u_j < 3; u_j++)
      {
        pz_historyPos_t->pd_historyPos[u_i][u_j] = pz_historyPos_t->pd_historyPos[u_i - 1][u_j];
        pz_historyPos_t->pd_SPPvelENU[u_i][u_j] = pz_historyPos_t->pd_SPPvelENU[u_i - 1][u_j];
      }
      pz_historyPos_t->pd_deltaTime[u_i] = pz_historyPos_t->pd_deltaTime[u_i - 1];
    }
    pz_historyPos_t->pd_deltaTime[0] = d_diffTime;
    for (u_j = 0; u_j < 3; u_j++)
    {
      pz_historyPos_t->pd_historyPos[0][u_j] = pz_filterSol->pd_filterPos[u_solTag][u_j];
    }
    pz_historyPos_t->u_num =
      (pz_historyPos_t->u_num >= MAX_NUM_HISTORY_POS ? MAX_NUM_HISTORY_POS : pz_historyPos_t->u_num + 1);
  }

  pz_historyPos_t->z_tor = z_tor;
  pz_historyPos_t->pu_posType[0] = u_solTag;

  if (NULL != pz_posSPP && tm_GpsTimeDiff(&z_tor, &(pz_posSPP->z_gpsTime)) < 1e-3)
  {
    for (u_j = 0; u_j < 3; u_j++)
    {
      pz_historyPos_t->pd_SPPvelENU[0][u_j] = pz_posSPP->f_velEnu[u_j];
    }
  }

  pz_historyPos_t->q_DiffFlagWithSPP <<= 2;
  if ((q_diffFlagWithSPP & 0x3) == 0x3)
  {
    pz_historyPos_t->q_DiffFlagWithSPP |= 0x3;
  }
  else if ((q_diffFlagWithSPP & 0x1) == 0x1)
  {
    pz_historyPos_t->q_DiffFlagWithSPP |= 0x1;
  }

  /*test*/
  //if (pz_historyPos_t->u_num == MAX_NUM_HISTORY_POS)
  //{
  //  pz_historyPos_t->pd_historyPos[0][0] = 2.0;
  //  pz_historyPos_t->pd_historyPos[1][0] = 5.0;
  //  pz_historyPos_t->pd_historyPos[2][0] = 10.0;
  //  pz_historyPos_t->pd_historyPos[3][0] = 17.0;
  //  pz_historyPos_t->pd_historyPos[4][0] = 26.0;
  //}
  //rtk_useHistoryPosEstimateCurrentPos(pz_historyPos_t, z_tor, &d_diffTime);

  return TRUE;
}

/**
 * @brief              get flag of whether current sol has significant difference with history predict one
 * @param[in]          pz_filterSol save position result of every steps in rtk process
 * @param[in]          u_solType see RTK_FILTER_POS_...
 * @return             RTK_DIFF_BETWEEN_POS_NONE for no diff or unknowen, RTK_DIFF_BETWEEN_POS_SMALL/LARGE for small/large diff
 */
uint8_t rtk_getDiffFlagBewteen_history_current(const rtk_filter_sol_t* pz_filterSol, uint8_t u_solType)
{
  uint8_t u_i = 0;
  uint8_t u_status = RTK_DIFF_BETWEEN_POS_NONE;
  const double* pd_pos = NULL;
  const double* pd_preditPos = NULL;
  double d_distanceVertical = 0.0;
  double d_distanceHorizontal = 0.0;
  double pd_deltaXYZ[3] = { 0.0 };
  double pd_lla[3] = { 0.0 };
  double pd_enu[3] = { 0.0 };
  char* u_posType[MAX_FILTER_POS_STATUS] = { "SPP","Predict","INS","RTD",
                                             "Float","PREFIX","FIX","FIX_WL" };
  /* threshold define begin */
  const double d_SmallDiffHori = 3.0;
  const double d_SmallDiffVert = 8.0;
  const double d_LargeDiffHori = 8.0;
  const double d_LargeDiffVert = 20.0;
  /* threshold define end */

  if (pz_filterSol == NULL)
  {
    return u_status;
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREDICT] != RTK_FILTER_SOL_STAT_VALID)
  {
    return u_status;
  }
  pd_preditPos = pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREDICT];
  if (gnss_Dot(pd_preditPos, pd_preditPos, 3) < FABS_ZEROS)
  {
    return u_status;
  }
  pd_pos = pz_filterSol->pd_filterPos[u_solType];

  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_deltaXYZ[u_i] = pd_preditPos[u_i] - pd_pos[u_i];
  }
  gnss_Ecef2Lla(pd_pos, pd_lla);
  gnss_Ecef2Enu(pd_lla, pd_deltaXYZ, pd_enu);

  d_distanceVertical = fabs(pd_enu[2]);
  d_distanceHorizontal = gnss_Norm(pd_enu, 2);

  if (d_distanceVertical > d_SmallDiffVert || d_distanceHorizontal > d_SmallDiffHori)
  {
    if (d_distanceVertical > d_LargeDiffVert || d_distanceHorizontal > d_LargeDiffHori)
    {
      u_status = RTK_DIFF_BETWEEN_POS_LARGE;
    }
    else
    {
      u_status = RTK_DIFF_BETWEEN_POS_SMALL;
    }
  }

  if (u_status != RTK_DIFF_BETWEEN_POS_NONE)
  {
    LOGI(TAG_HPP, " [ Delta between Predict & %-8s ] dis_h = %6.3f, dis_2d = %6.3f\n",
      u_posType[u_solType], d_distanceVertical, d_distanceHorizontal);
  }

  return u_status;

}

/**
 * @brief              get flag of whether rtk sol has significant difference with spp
 * @param[in]          pz_filterSol save position result of every steps in rtk process
 * @param[in]          pz_positionFix save spp result
 * @param[in]          u_solType see RTK_FILTER_POS_...
 * @return             RTK_DIFF_BETWEEN_POS_NONE for no diff or unknowen, RTK_DIFF_BETWEEN_POS_SMALL/LARGE for small/large diff
 */
uint8_t rtk_getDiffFlagBewteen_spp_rtk(const rtk_filter_sol_t* pz_filterSol,
  const gnss_PositionFix_t* pz_positionFix, uint8_t u_solType)
{
  uint8_t u_i = 0;
  uint8_t u_status = RTK_DIFF_BETWEEN_POS_NONE;
  const double* pd_pos = NULL;
  const double* pd_vel = NULL;
  double d_distanceVertical = 0.0;
  double d_distanceHorizontal = 0.0;
  double d_deltaVelVertical = 0.0;
  double d_deltaVelHorizontal = 0.0;
  double pd_deltaXYZ[3] = { 0.0 };
  double pd_deltaVel[3] = { 0.0 };
  double pd_enu[3] = { 0.0 };
  double pd_enuVel[3] = { 0.0 };
  char* u_posType[MAX_FILTER_POS_STATUS] = { "SPP","Predict","INS","RTD",
                                             "Float","PREFIX","FIX","FIX_WL" };
  /* threshold define begin */
  const double d_MaxSppUnc = 5.0;
  const double d_MaxSppVelUnc = 3.0;
  const double d_SmallDiffHori =  8.0;
  const double d_SmallDiffVert = 16.0;
  const double d_LargeDiffHori = 20.0;
  const double d_LargeDiffVert = 30.0;
  /* threshold define end */

  if (pz_filterSol == NULL)
  {
    return u_status;
  }

  for (u_i = 0; u_i < 3; u_i++) 
  {
    if (pz_positionFix->f_posLlaUnc[u_i] > d_MaxSppUnc || pz_positionFix->f_velEnuUnc[u_i] > d_MaxSppVelUnc)
    {
      return u_status;
    }
  }

  pd_pos = pz_filterSol->pd_filterPos[u_solType];
  pd_vel = pz_filterSol->pd_filterVel[u_solType];

  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_deltaXYZ[u_i] = pz_positionFix->d_xyz[u_i] - pd_pos[u_i];
    pd_deltaVel[u_i] = pz_positionFix->f_velXyz[u_i] - pd_vel[u_i];
  }
  gnss_Ecef2Enu(pz_positionFix->d_lla, pd_deltaXYZ, pd_enu);
  gnss_Ecef2Enu(pz_positionFix->d_lla, pd_deltaVel, pd_enuVel);

  d_distanceVertical = fabs(pd_enu[2]);
  d_distanceHorizontal = gnss_Norm(pd_enu, 2);
  d_deltaVelVertical = fabs(pd_enuVel[2]);
  d_deltaVelHorizontal = gnss_Norm(pd_enuVel, 2);

  // vel add in next version (update in future)
  if (d_distanceVertical> d_SmallDiffVert || d_distanceHorizontal > d_SmallDiffHori)
  {
    if (d_distanceVertical > d_LargeDiffVert || d_distanceHorizontal > d_LargeDiffHori)
    {
      u_status = RTK_DIFF_BETWEEN_POS_LARGE;
    }
    else
    {
      u_status = RTK_DIFF_BETWEEN_POS_SMALL;
    }
  }

  if (u_status != RTK_DIFF_BETWEEN_POS_NONE)
  {
    LOGI(TAG_HPP, " [ Delta between SPP & %-8s ] dis_h = %6.3f, dis_2d = %6.3f, dv_h = %6.3f, dv_2d = %6.3f\n",
      u_posType[u_solType], d_distanceVertical, d_distanceHorizontal, d_deltaVelVertical, d_deltaVelHorizontal);
  }

  return u_status;

}

/**
 * @brief              use History Pos Estimate Current Pos
 * @param[in]          save history position position
 * @param[in]          z_tor time of current epoch
 * @param[out]         pd_currentPos pos of estimated current pos
 * @return             TRUE for success ,FALSE for error case
 */
BOOL rtk_useHistoryPosEstimateCurrentPos(rtk_history_pos_t* pz_historyPos_t, const GpsTime_t z_tor, double* pd_currentPos)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_n = MAX_NUM_HISTORY_POS - 1;
  double d_vSTD = 0.0;
  double d_estAccuracy = 0.0;
  double d_vi = 0.0;
  double d_ai = 0.0;
  double d_sumt = 0.0;
  double d_suma = 0.0;
  double d_sumv = 0.0;
  double d_suma2 = 0.0;
  double d_sumva = 0.0;
  double d_vel_i = 0.0;
  double d_diffTime = 0.0;
  double d_deltaTime = 0.0;
  double d_totalTime = 0.0;
  double d_vel = 0.0;
  double d_acc = 0.0;
  double d_vel_halt[3] = { 0.0 };
  double d_acc_halt[3] = { 0.0 };
  double d_pos_halt[3] = { 0.0 };
  double d_pos_est[MAX_NUM_HISTORY_POS] = { 0.0 };
  double d_V[MAX_NUM_HISTORY_POS][3] = { {0} };
  /* threshold define begin */
  const double d_MaxVstd = 1.2; 
  const double d_MaxAccuracy = 1.5;
  const double d_errorCoefficientVel = 0.02;
  const double d_errorCoefficientAcc = 0.05;
  const double d_MaxDiffTime = 1.5; //max diff time
  /* threshold define end */

  if (pz_historyPos_t->u_num < MAX_NUM_HISTORY_POS || MAX_NUM_HISTORY_POS < 3 || NULL == pd_currentPos)
  {
    return FALSE;
  }
  d_diffTime = tm_GpsTimeDiff(&z_tor, &pz_historyPos_t->z_tor);
  if (d_diffTime > d_MaxDiffTime)
  {
    return FALSE;
  }
  for (u_i = 0; u_i < MAX_NUM_HISTORY_POS; u_i++)
  {
    if (pz_historyPos_t->pd_deltaTime[u_i] < FABS_ZEROS ||
      gnss_Dot(pz_historyPos_t->pd_historyPos[u_i], pz_historyPos_t->pd_historyPos[u_i], 3) < FABS_ZEROS * FABS_ZEROS)
    {
      break;
    }
  }
  if (u_i < MAX_NUM_HISTORY_POS)
  {
    return FALSE;
  }

  /**
   * consider as CA dynamic module, estimate vel & acc, parameter X=[ vel  acc ]'
   * x_i is history pos, t_i is delta time.
   * v_i = ( x_i+1 - x_i ) / t_i ;   a_i = t_i +...+ t_1 - t_i / 2 ;
   * L = [ v_1 v_2 v_3 v_4 ]'   H = [   -1   -1   -1   -1 ]'
   *                                [  a_1  a_2  a_3  a_4 ]
   * X = (H'H)^-1 * H'L = 1 / (4*sum(a*a)-sum(a)*sum(a)) * [ - sum(a*a)*sum(v) + sum(a)*sum(v*a) ]
   *                                                       [ -   sum(a)*sum(v) +      4*sum(v*a) ]
   * V = L - HX
   **/
  for (u_j = 0; u_j < 3; u_j++)
  {
    d_sumt = 0.0;
    d_suma = 0.0;
    d_sumv = 0.0;
    d_suma2 = 0.0;
    d_sumva = 0.0;
    for (u_i = 0; u_i < MAX_NUM_HISTORY_POS - 1; u_i++)
    {
      d_vi = (pz_historyPos_t->pd_historyPos[u_i + 1][u_j] - pz_historyPos_t->pd_historyPos[u_i][u_j])
        / pz_historyPos_t->pd_deltaTime[u_i];
      d_sumt += pz_historyPos_t->pd_deltaTime[u_i];
      d_ai = d_sumt - pz_historyPos_t->pd_deltaTime[u_i] / 2;
      d_suma += d_ai;
      d_sumv += d_vi;
      d_suma2 += d_ai * d_ai;
      d_sumva += d_vi * d_ai;
    }
    d_vel_halt[u_j] = (- d_suma2 * d_sumv + d_suma * d_sumva) / (u_n * d_suma2 - d_suma * d_suma);
    d_acc_halt[u_j] = (-  d_suma * d_sumv +    u_n * d_sumva) / (u_n * d_suma2 - d_suma * d_suma);
  }
  // calculate residual
  for (u_j = 0; u_j < 3; u_j++) 
  {
    d_sumt = 0.0;
    for (u_i = 0; u_i < MAX_NUM_HISTORY_POS - 1; u_i++)
    {
      d_vi = (pz_historyPos_t->pd_historyPos[u_i + 1][u_j] - pz_historyPos_t->pd_historyPos[u_i][u_j])
        / pz_historyPos_t->pd_deltaTime[u_i];
      d_sumt += pz_historyPos_t->pd_deltaTime[u_i];
      d_ai = d_sumt - pz_historyPos_t->pd_deltaTime[u_i] / 2;
      d_V[u_i][u_j] = d_vi + d_vel_halt[u_j] - d_ai * d_acc_halt[u_j];
      d_vSTD += fabs(d_V[u_i][u_j]);
    }
  }
  d_vSTD = d_vSTD / (3.0 * u_n);
  if (d_vSTD > d_MaxVstd)
  {
    return FALSE;
  }

  // Get vel & acc, estimate accurary of current pos
  d_estAccuracy = 0;
  for (u_j = 0; u_j < 3; u_j++)
  {
    d_deltaTime = 0.0;
    for (u_i = 0; u_i < MAX_NUM_HISTORY_POS; u_i++)
    {
      d_totalTime = d_diffTime + d_deltaTime;
      d_vel_i = d_vel_halt[u_j] - d_deltaTime * d_acc_halt[u_j];
      d_pos_est[u_i] = pz_historyPos_t->pd_historyPos[u_i][u_j] + d_vel_i * d_totalTime +
        d_acc_halt[u_j] * d_totalTime * d_totalTime / 2;
      d_pos_halt[u_j] += d_pos_est[u_i];
      d_deltaTime += pz_historyPos_t->pd_deltaTime[u_i];
    }
    d_pos_halt[u_j] = d_pos_halt[u_j] / MAX_NUM_HISTORY_POS;
    for (u_i = 0; u_i < MAX_NUM_HISTORY_POS; u_i++)
    {
      d_estAccuracy += fabs(d_pos_est[u_i] - d_pos_halt[u_j]);
    }
    pd_currentPos[u_j] = d_pos_halt[u_j];
  }
  d_estAccuracy = d_estAccuracy / (3.0 * u_n);
  //add vel & acc estimate error
  d_vel = gnss_Norm(d_vel_halt, 3);
  d_acc = gnss_Norm(d_acc_halt, 3);
  d_estAccuracy += d_vSTD;
  d_estAccuracy += fabs(d_errorCoefficientVel * d_vel * d_diffTime)
    + fabs(d_errorCoefficientAcc * d_acc * d_diffTime * d_diffTime);
  if (d_estAccuracy > d_MaxAccuracy)
  {
    return FALSE;
  }
  return TRUE;
}

/**
 * @brief              get Predict Pos By History Pos
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]          z_tor time of current epoch
 * @return             TRUE for success ,FALSE for error case
 */
BOOL rtk_getPredictPosByHistoryPos(rtk_filterInfo_t* pz_filterInfo, const GpsTime_t z_tor)
{
  uint8_t u_i;
  BOOL b_status = FALSE;
  double pd_currentPos[3] = { 0.0 };
  if (NULL != pz_filterInfo->pz_historyPos_t && NULL != pz_filterInfo->pz_filterSol &&
    rtk_useHistoryPosEstimateCurrentPos(pz_filterInfo->pz_historyPos_t, z_tor, pd_currentPos))
  {
    for (u_i = 0; u_i < 3; u_i++)
    {
      pz_filterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREDICT][u_i] = pd_currentPos[u_i];
    }
    pz_filterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_PREDICT] = RTK_FILTER_SOL_STAT_VALID;
    b_status = TRUE;
  }
  return b_status;
}
/**
 * @brief fill RTK position information
 * @param[in] pz_satSigMeasCollect is the observation information
 * @param[in/out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_fixedAmbPool input information for RTK ambiguity resolution
 * @param[out] pz_posSolution position struct contains pos/vel and unc of pos/vel
 * @param[out] pz_preFixedAmbPool prefixed ambiguity pool
 * return
 */
void rtk_CreatePosSolution(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, rtk_filterInfo_t* pz_RTKfilterInfo,
  const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_PositionFix_t* pz_posSolution, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_sys = 0;
  uint8_t u_wl = 0;
  uint8_t u_gps = 0;
  uint16_t u_solTag = MAX_FILTER_POS_STATUS;
  double pd_x[3] = { 0.0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  const gnss_PositionFix_t* pz_posSPP = &pz_satSigMeasCollect->z_positionFix;
  int16_t pw_index[3] = { 0, 1, 2 };
  rtk_filter_sol_t* pz_filterSol;
  char* u_posType[MAX_FILTER_POS_STATUS] = { "SPP","Predict","INS","RTD",
                                             "Float","PREFIX","FIX","FIX_WL","FIX_EWL" };
  char* u_posStatType[MAX_RTK_FILTER_SOL_STAT] = { "NONE","SAVE","RESET","ERROR","WARN","VALID" };

  pz_filterSol = pz_RTKfilterInfo->pz_filterSol;
  if (NULL == pz_RTKfilterInfo || NULL == pz_filterSol)
  {
    return;
  }

  // Position solution source from PPP engine
  pz_posSolution->u_fixSource = FIX_SOURCE_RTK;
  pz_posSolution->z_gpsTime = pz_posSPP->z_gpsTime;
  pz_posSolution->w_size = pz_posSPP->w_size;
  // PVT
  if (FALSE == pz_RTKfilterInfo->q_filterPosValid)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_NONE;
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_NONE;
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_NONE;
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_WL] = RTK_FILTER_SOL_STAT_NONE;
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_EWL] = RTK_FILTER_SOL_STAT_NONE;
  }

  double* d_QEcefPos = OS_MALLOC(sizeof(double) * 3 * 3);
  double* d_QenuPos = OS_MALLOC(sizeof(double) * 3 * 3);
  if (any_Ptrs_Null(2, d_QEcefPos, d_QenuPos))
  {
    OS_FREE(d_QEcefPos);
    OS_FREE(d_QenuPos);
    *pz_posSolution = pz_satSigMeasCollect->z_positionFix;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTK;
    return;
  }
  // Print filter pos
  LOGI(TAG_HPP, "-------------------TOW OF FILTER POS : %9.2f-------------------\n",
    pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV);
  for (u_i = 0; u_i < MAX_FILTER_POS_STATUS; u_i++)
  {
    if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[u_i] > RTK_FILTER_SOL_STAT_NONE)
    {
      LOGI(TAG_HPP, " < FILTER POS %-8s> %-5s %12.3f,%12.3f,%12.3f\n", u_posType[u_i],
        u_posStatType[pz_RTKfilterInfo->pz_filterSol->pu_posValid[u_i]],
        pz_RTKfilterInfo->pz_filterSol->pd_filterPos[u_i][0],
        pz_RTKfilterInfo->pz_filterSol->pd_filterPos[u_i][1],
        pz_RTKfilterInfo->pz_filterSol->pd_filterPos[u_i][2]);
    }
  }
  LOGI(TAG_HPP, "----------------------END OF FILTER POS PRINT----------------------\n");

  //LOGW for realtime print out
  //LOGW is not encoded and it's exposed to clients, so could not be too comprehensive in writing
  //See RTK_FILTER_POS... SPP | Predict Ins_machinery | RTD Float | Pre_Fix Fix | Fix_WL Fix_EWL |
  LOGW(TAG_HPP, "--SOLS--| %d |%d %d|%d %d|%d %d|%d %d|\n", pz_filterSol->pu_posValid[RTK_FILTER_POS_SPP],
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREDICT], pz_filterSol->pu_posValid[RTK_FILTER_POS_INS],
    pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD], pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT],
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX], pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX],
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_WL], pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_EWL]);

  // decide out sol stat
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_VALID)
  {
    u_solTag = RTK_FILTER_POS_FIX;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] >= RTK_FILTER_SOL_STAT_VALID)
  {
    u_solTag = RTK_FILTER_POS_PREFIX;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_VALID &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_WL] >= RTK_FILTER_SOL_STAT_VALID)
  {
    u_solTag = RTK_FILTER_POS_FIX_WL;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_VALID &&
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX_EWL] >= RTK_FILTER_SOL_STAT_VALID)
  {
    u_solTag = RTK_FILTER_POS_FIX_EWL;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN)
  {
    u_solTag = RTK_FILTER_POS_FLOAT;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] >= RTK_FILTER_SOL_STAT_VALID)
  {
    u_solTag = RTK_FILTER_POS_RTD;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_SPP] >= RTK_FILTER_SOL_STAT_SAVE)
  {
    u_solTag = RTK_FILTER_POS_SPP;
  }

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN)
  {
    pz_RTKfilterInfo->w_filterWithPhaseCount++;
    pz_RTKfilterInfo->w_rtdCount = 0;
  }
  else
  {
    pz_RTKfilterInfo->w_filterWithPhaseCount = 0;
    pz_RTKfilterInfo->w_rtdCount++;
  }

  if (u_solTag == RTK_FILTER_POS_FIX || u_solTag == RTK_FILTER_POS_FIX_WL || u_solTag == RTK_FILTER_POS_PREFIX ||
    u_solTag == RTK_FILTER_POS_FLOAT || u_solTag == RTK_FILTER_POS_RTD || u_solTag == RTK_FILTER_POS_FIX_EWL)
  {
    rtk_pushHistoryPos(pz_RTKfilterInfo, u_solTag, pz_satSigMeasCollect->z_tor, pz_filterSol->pq_diffFlagWithSPP[u_solTag], pz_posSPP);
  }

  if (u_solTag == RTK_FILTER_POS_FLOAT || u_solTag == RTK_FILTER_POS_RTD)
  {
    pz_RTKfilterInfo->w_floatCount++;
  }

  if (u_solTag == RTK_FILTER_POS_FIX || u_solTag == RTK_FILTER_POS_FIX_WL
    || u_solTag == RTK_FILTER_POS_PREFIX || u_solTag == RTK_FILTER_POS_FIX_EWL)
  {
    pz_posSolution->u_fixFlag = GNSS_FIX_FLAG_FIXED;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTK_FIX;
    pz_RTKfilterInfo->w_floatCount = 0;
  }
  else if (u_solTag == RTK_FILTER_POS_FLOAT)
  {
    pz_posSolution->u_fixFlag = GNSS_FIX_FLAG_FLOATING;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTK_FLOAT;
  }
  else if (u_solTag == RTK_FILTER_POS_RTD)
  {
    pz_posSolution->u_fixFlag = GNSS_FIX_FLAG_DGNSS;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTD;
  }
  else if (u_solTag == RTK_FILTER_POS_SPP)
  {
    pz_posSolution->u_fixFlag = GNSS_FIX_FLAG_SPS;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTK;
  }
  else
  {
    pz_posSolution->u_fixFlag = GNSS_FIX_FLAG_INVALID;
    pz_posSolution->u_fixSource = FIX_SOURCE_RTK;
  }

  pz_RTKfilterInfo->u_solTag = (uint8_t)u_solTag;

  if (pz_posSolution->u_fixFlag != GNSS_FIX_FLAG_INVALID)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_x[u_i] = pz_filterSol->pd_filterPos[u_solTag][u_i];
    }

    // Position Ecef
    pz_posSolution->d_xyz[0] = pd_x[0];
    pz_posSolution->d_xyz[1] = pd_x[1];
    pz_posSolution->d_xyz[2] = pd_x[2];
    gnss_Ecef2Lla(pz_posSolution->d_xyz, pz_posSolution->d_lla);
    pz_posSolution->f_velXyz[0] = pz_posSPP->f_velXyz[0];
    pz_posSolution->f_velXyz[1] = pz_posSPP->f_velXyz[1];
    pz_posSolution->f_velXyz[2] = pz_posSPP->f_velXyz[2];
    pz_posSolution->f_velEnu[0] = pz_posSPP->f_velEnu[0];
    pz_posSolution->f_velEnu[1] = pz_posSPP->f_velEnu[1];
    pz_posSolution->f_velEnu[2] = pz_posSPP->f_velEnu[2];
    // uncertainty pos lla
    pz_posSolution->f_posLlaUnc[0] = pz_posSPP->f_posLlaUnc[0];
    pz_posSolution->f_posLlaUnc[1] = pz_posSPP->f_posLlaUnc[1];
    pz_posSolution->f_posLlaUnc[2] = pz_posSPP->f_posLlaUnc[2];
    // ENU Q
    d_QenuPos[0] = (double)pz_posSolution->f_posLlaUnc[0];
    d_QenuPos[4] = (double)pz_posSolution->f_posLlaUnc[1];
    d_QenuPos[8] = (double)pz_posSolution->f_posLlaUnc[2];
    gnss_CovEnu2Ecef(pz_posSolution->d_lla, d_QenuPos, d_QEcefPos);
    // uncertainty pos ecef
    pz_posSolution->f_posXyzUnc[0] = (float)sqrt(d_QEcefPos[0]);
    pz_posSolution->f_posXyzUnc[1] = (float)sqrt(d_QEcefPos[4]);
    pz_posSolution->f_posXyzUnc[2] = (float)sqrt(d_QEcefPos[8]);
    // uncertainty vel enu
    pz_posSolution->f_velEnuUnc[0] = pz_posSPP->f_velEnuUnc[0];
    pz_posSolution->f_velEnuUnc[1] = pz_posSPP->f_velEnuUnc[1];
    pz_posSolution->f_velEnuUnc[2] = pz_posSPP->f_velEnuUnc[2];

    pz_posSolution->z_dops = pz_satSigMeasCollect->z_positionFix.z_dops;
    tm_cvt_GpstToEpoch(&pz_satSigMeasCollect->z_tor, &pz_posSolution->z_epoch);
    pz_posSolution->u_leapsec = tm_LeapSecond_FromEpoch(&pz_posSolution->z_epoch);
    pz_posSolution->u_CN040 = pz_posSPP->u_CN040;
    pz_posSolution->d_avgCN0 = pz_posSPP->d_avgCN0;
    pz_posSolution->d_quasiGeoidHeight = pz_posSPP->d_quasiGeoidHeight;
    pz_posSolution->z_rtk.f_age = pz_RTKfilterInfo->f_age;
    pz_posSolution->z_rtk.q_StationID = pz_RTKfilterInfo->q_StationID;
    memcpy(pz_posSolution->z_rtk.pd_StationCoordinate, pz_RTKfilterInfo->pd_StationCoordinate, sizeof(pz_RTKfilterInfo->pd_StationCoordinate));
  }
  memcpy(&pz_posSolution->z_SvStatus, &pz_posSPP->z_SvStatus, sizeof(gnss_PositionFix_SvStatus_t));

  OS_FREE(d_QEcefPos);
  OS_FREE(d_QenuPos);
  return;
}
