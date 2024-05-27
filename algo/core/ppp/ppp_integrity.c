/**@file        ppp_integrity.cpp
 * @brief       Calculate PL for PPP FLOAT/FIX
 * @details     
 * @author      houxiaowei
 * @date        2023/2/23
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/2/23   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "integ_type.h"
#include "ppp_integrity.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "gnss_common.h"
#include "ppp_type.h"
#include "ppp_integ_model.h"

static integ_Config_t gz_int_Config;
static integ_InfoBlock_t* gpz_int_InfoBlock;
static integ_PppFeature_t* gpz_pppFeature;
static integ_Uncsmt_t ppp_uncsmt = {0};

static double d_ppf[] =
  {1.644853626951, 2.575829303549, 3.290526731492, 3.890591886413, 4.417173413469, 4.891638475699, 5.326723886384
    // 0.9,1        0.99,2          0.999,3         0.9999,4        0.99999,5       0.999999,6      0.9999999,7
  };

#define  INTEGRITY_BETA    (0.4)
#define  INTEGRITY_RO      (0.0)  //TODO(houxiaowei): Rm_enu[4]<0 when seted 0.1

void ppp_IntegInit()
{
  M_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_SET);  // use integrity
  gpz_pppFeature = NULL;
  gpz_int_InfoBlock = NULL;

  gpz_pppFeature = OS_MALLOC_FAST(sizeof(integ_PppFeature_t));
  gpz_int_InfoBlock = OS_MALLOC_FAST(sizeof(integ_InfoBlock_t));
  if (any_Ptrs_Null(2, gpz_pppFeature, gpz_int_InfoBlock))
  {
    OS_FREE(gpz_pppFeature);
    OS_FREE(gpz_int_InfoBlock);
    return;
  }
  /* Function to cal integrity */
  gz_int_Config.u_fun_type = C_INTEGRITY_FUN_DECISION_TREE;

  /* init suc */
  M_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT);
}

void ppp_IntegDeInit()
{
  OS_FREE(gpz_int_InfoBlock);
  OS_FREE(gpz_pppFeature);
}

/**
 * @brief Get integrity, get protection level by "cov" or "filter" or "fusing"
 * @param[in/out] pz_pl protection level result
 */
void ppp_IntegFillUpIntegrity(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl)
{
  if (!ppp_IntegValid())
  {
    return;
  }
  switch (gz_int_Config.u_fun_type)
  {
  case C_INTEGRITY_FUN_FILTER:
    ppp_IntegCalPlByFilter(pz_posSolution, pz_pl);
    break;
  case C_INTEGRITY_FUN_RAIM:
    ppp_IntegCalPLByRAIM(pz_pl);
    break;
  case C_INTEGRITY_FUN_DECISION_TREE:
    ppp_IntegCalPLByDecisionTree(pz_posSolution, pz_pl);
    break;
  default:
    ppp_IntegCalPlByFilter(pz_posSolution, pz_pl);
    break;
  }
}

void ppp_IntegClear(const GpsTime_t* gpst)
{
  if (!M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT))
  {
    return;
  }
  memset(gpz_int_InfoBlock, 0, sizeof(integ_InfoBlock_t));
  gpz_int_InfoBlock->z_stored.d_nm1 = 0.1;
  gpz_int_InfoBlock->z_tor = *gpst;

  memset(gpz_pppFeature, 0, sizeof(integ_PppFeature_t));
  gpz_pppFeature->z_tor = *gpst;
}

integ_FunType ppp_IntegGetIntegrityType()
{
  return gz_int_Config.u_fun_type;
}

/**
 * @brief Intergrity status
 * @return TRUE: ok, False: not ok
 */
BOOL ppp_IntegValid()
{
  if (M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT) &&
    M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_SET))
  {
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief Set position engine status
 * @param[in] u_fix_flag
 */
void ppp_IntegSetPosStatus(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FixFlagType u_fix_flag)
{
  if (!M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT))
  {
    return;
  }
  gpz_int_InfoBlock->u_fixFlag = u_fix_flag;
  gpz_pppFeature->u_fixFlag = u_fix_flag;
}

/**
 * @brief get the num of observation whos CN0 > 40
 * @param[in] gnss_SatSigMeasCollect_t* pz_satSigMeasCollect
 */
void ppp_IntegSetCN040(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  gpz_pppFeature->u_CN040 = 0;
  uint8_t u_satMeasCount = pz_satSigMeasCollect->u_satMeasCount;
  for (uint8_t u_i = 0; u_i < u_satMeasCount; u_i++)
  {
    uint8_t satIdx = pz_satSigMeasCollect->u_satMeasIdxTable[u_i];
    gnss_SatelliteMeas_t* satMeas = pz_satSigMeasCollect->pz_satMeas[satIdx];
    if (satMeas == NULL)
    {
      continue;
    }
    for (uint8_t u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      gnss_SignalMeas_t* sigMeas = satMeas->pz_signalMeas[u_j];
      if (sigMeas == NULL)
      {
        continue;
      }
      if (sigMeas->f_cn0 >= 40)
      {
        gpz_pppFeature->u_CN040++;
      }
    }
  }
}

/**
 * @brief Set position rotation matrix
 * @param[in] d_pos LLA, {lat,lon,height}
 */
void ppp_IntegSetPosAndPosRotation(const double d_pos[3])
{
  if (!M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT))
  {
    return;
  }
  // Set pos & neu rotation matrix
  for (uint8_t i = 0; i < 3; ++i)
  {
    gpz_int_InfoBlock->d_lla[i] = d_pos[i];
  }

  // gnss_Pos2EnuMat_row(d_pos, gpz_int_InfoBlock->d_neuRotation);
  gnss_Pos2EnuMat(d_pos, gpz_int_InfoBlock->d_neuRotation);
}

/**
 * @brief Calculate DOPS by filter the invalid satellites
 * @param[in] pz_SatSigMeasCollect
 * @param[in] pz_filterObs
 */
void ppp_IntegCalSetDOPS(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, const ppp_EpochFilterObs_t* pz_filterObs)
{
  int sat_flag = 0, n = 0;
  matrix_t* mH = matrix_new(pz_SatSigMeasCollect->u_satMeasCount, 4);
  matrix_t* mHT = matrix_new(pz_SatSigMeasCollect->u_satMeasCount, 4);
  matrix_t* mQ = matrix_new(4, 4);
  matrix_t* mQI = matrix_new(4, 4);
  if (mH == NULL || mHT == NULL || mQ == NULL || mQI == NULL)
  {
    matrix_free(&mH);
    matrix_free(&mHT);
    matrix_free(&mQ);
    matrix_free(&mQI);
    return;
  }

  gnss_SatelliteMeas_t* pz_meas = NULL;

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    sat_flag = 0;
    pz_meas = pz_SatSigMeasCollect->pz_satMeas[i];
    if (pz_meas == NULL || pz_meas->z_satPosVelClk.f_elevation <= 0.0 || pz_meas->z_satPosVelClk.f_azimuth <= 0)
    {
      continue;
    }
    for (int j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      if (pz_filterObs->pz_SatFilterObs[i]->z_measUpdateFlag[j] == NULL ||
        pz_filterObs->pz_SatFilterObs[i]->z_measUpdateFlag[j]->b_valid == 0)
      {
        continue;
      }
      sat_flag = 1;
    }
    if (sat_flag == 1)
    {
      double cosel = cos((double)pz_meas->z_satPosVelClk.f_elevation);
      double sinel = sin((double)pz_meas->z_satPosVelClk.f_elevation);
      MAT(mH, n, 0) = -cosel * sin((double)pz_meas->z_satPosVelClk.f_azimuth);
      MAT(mH, n, 1) = -cosel * cos((double)pz_meas->z_satPosVelClk.f_azimuth);
      MAT(mH, n, 2) = -sinel;
      MAT(mH, n, 3) = 1.0;
      n++;
    }
    if (n >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
  }
  if ((n >= 4) && matrix_cpy(mHT, mH))
  {
    matrix_transpose(mHT);
    matrix_mul(mHT, mH, mQ);
    if (matrix_inverse(mQ, mQI))
    {
      gpz_int_InfoBlock->z_dops.f_gdop = (float)sqrt(
        MAT(mQI, 0, 0) + MAT(mQI, 1, 1) + MAT(mQI, 2, 2) + MAT(mQI, 3, 3));
      gpz_int_InfoBlock->z_dops.f_pdop = (float)sqrt(MAT(mQI, 0, 0) + MAT(mQI, 1, 1) + MAT(mQI, 2, 2));
      gpz_int_InfoBlock->z_dops.f_hdop = (float)sqrt(MAT(mQI, 0, 0) + MAT(mQI, 1, 1));
      gpz_int_InfoBlock->z_dops.f_vdop = (float)sqrt(MAT(mQI, 2, 2));
    }
  }
  matrix_free(&mH);
  matrix_free(&mHT);
  matrix_free(&mQ);
  matrix_free(&mQI);
}

static void
int_SaveHKFromSeqKalmanFilter(const SeqKalmanVar_t* pz_seqVar, const int16_t pw_PVAindex[9], double H[3], double K[3])
{
  for (uint16_t i = 0; i < pz_seqVar->w_n; ++i)
  {
    for (uint16_t j = 0; j < P_NUM; ++j)
    {
      if (pz_seqVar->pw_L[i] == pw_PVAindex[j])
      {
        H[j] = pz_seqVar->pd_h[i];
        int16_t w_idx = pz_seqVar->pw_L[i];
        if ((int8_t)INVALID_INDEX == w_idx)
        {
          continue;
        }
        K[j] = pz_seqVar->pd_k[w_idx];
      }
    }
  }
}

static double int_CalEffective(const SeqKalmanVar_t* pz_seqVar)
{
  double d_effective = 0.0;
  for (uint16_t i = 0; i < pz_seqVar->w_n; ++i)
  {
    uint16_t w_idx = pz_seqVar->pw_L[i];
    if ((uint16_t)INVALID_INDEX == w_idx)
    {
      continue;
    }
    d_effective += pz_seqVar->pd_h[i] * pz_seqVar->pd_k[w_idx];
    LOGI(TAG_PPP, "eff:%f, h:%f, k:%f\n", d_effective, pz_seqVar->pd_h[i], pz_seqVar->pd_k[w_idx]);
  }
  return d_effective;
}

/**
 * @brief Extract information from sequential EKF
 * @param[in] z_freqType
 * @param[in] pz_seqVar
 * @param[in] pw_PVAindex
 * @param[in,out] pz_si signal information
 */
void ppp_IntegExtractInfoFromSeqKalmanFilter(gnss_FreqType z_freqType, const SeqKalmanVar_t* pz_seqVar,
                                             const int16_t pw_PVAindex[9], integ_SignalInfo_t* pz_si)
{
  pz_si->u_frq = z_freqType;
  pz_si->u_valid = 1;

  // H[3], K[3]
  int_SaveHKFromSeqKalmanFilter(pz_seqVar, pw_PVAindex, pz_si->pd_H, pz_si->pd_K);

  // R, d_pz
  pz_si->d_R = pz_seqVar->d_R;
  pz_si->d_pz = pz_seqVar->d_pZ;

  // H@K
  pz_si->d_effective = int_CalEffective(pz_seqVar);

}

/**
 * @brief Save signal information that needed EKF variable
 * @param[in] u_constellation
 * @param[in] u_svid
 * @param[in] z_freqType
 * @param[in] pz_si signal info
 */
void ppp_IntegAddSignalInfo(gnss_ConstellationType u_constellation, uint8_t u_svid, gnss_FreqType z_freqType,
                            const integ_SignalInfo_t* pz_si)
{
  /* find sat's signal idx */
  int16_t q_idx = (int16_t)INVALID_INDEX;
  for (uint8_t u_i = 0; u_i < gpz_int_InfoBlock->u_satMeasCount; ++u_i)
  {
    if ((u_constellation == gpz_int_InfoBlock->z_satIntInfo[u_i].u_constellation) &&
      (u_svid == gpz_int_InfoBlock->z_satIntInfo[u_i].u_svid))
    {
      q_idx = u_i;
      break;
    }
  }
  if (q_idx == INVALID_INDEX)
  {
    q_idx = gpz_int_InfoBlock->u_satMeasCount;
  }

  /* assignment */
  gpz_int_InfoBlock->z_satIntInfo[q_idx].z_signalInfo[z_freqType] = *pz_si;
  if (q_idx == gpz_int_InfoBlock->u_satMeasCount)
  {
    gpz_int_InfoBlock->z_satIntInfo[q_idx].u_constellation = u_constellation;
    gpz_int_InfoBlock->z_satIntInfo[q_idx].u_svid = u_svid;
    gpz_int_InfoBlock->u_satMeasCount++;
  }
}

/**
 * @brief Add observation residual of after EKF
 * @param[in] b_fixed
 * @param[in] u_constellation
 * @param[in] u_svid
 * @param[in] z_freqType
 * @param[in] d_res
 * note: must call this function after ppp_IntegAddSignalInfo
 */
void ppp_IntegAddObsRes(BOOL b_fixed, gnss_ConstellationType u_constellation, uint8_t u_svid, gnss_FreqType z_freqType,
                        double d_res)
{

  int16_t q_idx = (int16_t)INVALID_INDEX;
  for (uint8_t u_i = 0; u_i < gpz_int_InfoBlock->u_satMeasCount; ++u_i)
  {
    if ((u_constellation == gpz_int_InfoBlock->z_satIntInfo[u_i].u_constellation) &&
      (u_svid == gpz_int_InfoBlock->z_satIntInfo[u_i].u_svid))
    {
      q_idx = u_i;
      break;
    }
  }
  if (q_idx == INVALID_INDEX)
  {
    return;
  }

  if (b_fixed)
  {
    gpz_int_InfoBlock->z_satIntInfo[q_idx].z_signalInfo[z_freqType].d_obs_res_fix = d_res;
  }
  else
  {
    gpz_int_InfoBlock->z_satIntInfo[q_idx].z_signalInfo[z_freqType].d_obs_res = d_res;
  }
}

static void int_t1_edited(integ_SignalInfo_t* pz_si, double Rm1[9], double* d_nm1, double* d_rm2, double* d_weight)
{
  double d_nm1t = .0;
  double d_rm2t = .0;

  /* magnification factor of the weight */
  d_nm1t = 1 - pz_si->d_effective + INTEGRITY_BETA * (*d_nm1);
  // if (d_nm1t < 0.01)  d_nm1t = 0.1;
  d_rm2t = pz_si->d_obs_res * pz_si->d_obs_res / pz_si->d_pz;   // weighted by std  (v^2 / sigma^2)
  d_rm2t = d_rm2t + INTEGRITY_BETA * (*d_nm1) * (*d_rm2);
  d_rm2t = d_rm2t / d_nm1t;


  /* K @ K' * d_pz * rm2, P after gain */
  for (size_t i = 0; i != 3; ++i)
  {
    for (size_t j = 0; j != 3; ++j)
    {
      Rm1[i + j * 3] = pz_si->pd_K[i] * pz_si->pd_K[j] * pz_si->d_pz;
    }
  }
  // matprint_e(Rm1, 3, 3, 12, 6);
  // printf("Rm1\n");
  // fflush(stdout);

  *d_weight = d_rm2t / pz_si->d_pz;
  *d_nm1 = d_nm1t;
  *d_rm2 = d_rm2t;
}

static void int_t2_edited(const integ_SignalInfo_t* pz_si, double d_U[9], double Rm2[9])
{
  for (uint8_t u_i = 0; u_i < 3; ++u_i)
  {
    for (uint8_t u_j = 0; u_j < 3; ++u_j)
    {
      d_U[u_j + u_i * 3] = -pz_si->pd_K[u_j] * pz_si->pd_H[u_i];
      if (u_i == u_j)
      {
        d_U[u_j + u_i * 3] = 1 + d_U[u_j + u_i * 3];
      }
    }
  }
  double d_R[9] = {0.0};
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, d_U, gpz_int_InfoBlock->z_stored.d_Rm, 0.0, d_R);
  gnss_MatrixMultiply("NT", 3, 3, 3, 1.0, d_R, d_U, 0.0, Rm2);
}

static void int_sum_edited(const double d_Rm1[9], const double d_Rm2[9], double d_Rm[9], double d_Am[3])
{
  double d_Dm[9] = {0.0};
  double d_F[3] = {0.0};
  double d_U[9] = {0.0};
  double d_Rm1_diagonal[3] = {sqrt(d_Rm1[0]), sqrt(d_Rm1[4]), sqrt(d_Rm1[8])};
  memcpy(d_U, gpz_int_InfoBlock->z_stored.d_U, sizeof(double) * 9);

  gnss_MatrixMultiply("NT", 1, 3, 3, 1.0, d_Am, d_U, 0.0, d_F);
  gnss_MatrixMultiply("NN", 3, 3, 1, 1.0, d_Rm1_diagonal, d_F, 0.0, d_Dm);

  for (size_t i = 0; i != 3; ++i)
  {
    for (size_t j = 0; j != 3; ++j)
    {
      d_Rm[j + i * 3] = d_Rm1[j + i * 3] + d_Rm2[j + i * 3] + d_Dm[j + i * 3] + d_Dm[i + j * 3];
    }
  }

  for (size_t i = 0; i != 3; ++i)
  {
    d_Am[i] = INTEGRITY_RO * (d_Rm1_diagonal[i] + d_F[i]);
  }
}

void int_measUpdate(const integ_SatInfo_t* pz_satIntInfo, integ_SignalInfo_t* pz_si, integ_PLResult_t* pz_int_pl,
                    integ_Stored_t* pz_stored)
{
  char satid[4] = {0};
  double Rm1[9] = {0};
  double Rm2[9] = {0};
  double Rm_enu[9] = {0};
  float f_xyzpl_signal[3] = {0};
  float f_neupl_signal[3] = {0};

  svid_SatString(pz_satIntInfo->u_svid, pz_satIntInfo->u_constellation, satid);

  /* calculation */
  int_t1_edited(pz_si, Rm1, &pz_stored->d_nm1, &pz_stored->d_rm2, &pz_si->d_weight);
  int_t2_edited(pz_si, pz_stored->d_U, Rm2);
  int_sum_edited(Rm1, Rm2, pz_stored->d_Rm, pz_stored->d_Am);
  // int_t1(pz_si, Rm1, &pz_stored->d_nm1, &pz_stored->d_rm2, &pz_si->d_weight);
  // int_t2(pz_si, pz_stored->d_U, Rm2);
  // int_sum(Rm1, Rm2, pz_stored->d_Rm, pz_stored->d_Am);

  // LOGI(TAG_PPP, "%s,frq:%d,eff:%f,weight=%f,rm2:%f,d_pz:%f\n", satid, pz_si->u_frq, pz_si->d_effective,
  //      pz_si->d_weight,
  //      gpz_int_InfoBlock->z_stored.d_rm2, pz_si->d_pz);
  // log_fflush();


  /*  pl */
  gnss_CovEcef2Enu(gpz_int_InfoBlock->d_lla, pz_stored->d_Rm, Rm_enu);
  f_xyzpl_signal[0] = (float)(sqrt(pz_stored->d_Rm[0]) * pz_si->d_weight);
  f_xyzpl_signal[1] = (float)(sqrt(pz_stored->d_Rm[4]) * pz_si->d_weight);
  f_xyzpl_signal[2] = (float)(sqrt(pz_stored->d_Rm[8]) * pz_si->d_weight);
  f_neupl_signal[0] = (float)(sqrt(Rm_enu[0]) * pz_si->d_weight);
  f_neupl_signal[1] = (float)(sqrt(Rm_enu[4]) * pz_si->d_weight);
  f_neupl_signal[2] = (float)(sqrt(Rm_enu[8]) * pz_si->d_weight);

  pz_int_pl->f_xyz_protection_level[0] += f_xyzpl_signal[0];
  pz_int_pl->f_xyz_protection_level[1] += f_xyzpl_signal[1];
  pz_int_pl->f_xyz_protection_level[2] += f_xyzpl_signal[2];
  pz_int_pl->f_enu_protection_level[0] += f_neupl_signal[0];
  pz_int_pl->f_enu_protection_level[1] += f_neupl_signal[1];
  pz_int_pl->f_enu_protection_level[2] += f_neupl_signal[2];

  LOGD(TAG_PPP, "PL Rm[0]: %.2f, Rm1[0]:%.2f, Rm2[0]:%.2f, weight:%.1f\n",
       sqrt(pz_stored->d_Rm[0]),
       sqrt(Rm1[0]),
       sqrt(Rm2[0]),
       pz_si->d_weight);
}

/**
 * @brief Calculate protection level by RAIM, need to optimize.
 * @param[in,out] pz_int_pl protection level result
 */
void ppp_IntegCalPLByRAIM(integ_PLResult_t* pz_int_pl)
{
  double d_weight_all = .0;
  integ_SignalInfo_t* pz_si = NULL;

  /* Update protective level */
  for (uint8_t u_i = 0; u_i < gpz_int_InfoBlock->u_satMeasCount; ++u_i)
  {
    for (uint8_t u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; ++u_j)
    {
      pz_si = &gpz_int_InfoBlock->z_satIntInfo[u_i].z_signalInfo[u_j];
      if (!pz_si->u_valid)
      {
        continue;
      }
      int_measUpdate(&gpz_int_InfoBlock->z_satIntInfo[u_i], pz_si, pz_int_pl, &gpz_int_InfoBlock->z_stored);
      d_weight_all += pz_si->d_weight;
    }
  }

  /* Weighted protection level */
  if (d_weight_all != 0)
  {
    for (uint8_t u_i = 0; u_i < 3; ++u_i)
    {
      pz_int_pl->f_xyz_protection_level[u_i] /= (float)d_weight_all;
      pz_int_pl->f_enu_protection_level[u_i] /= (float)d_weight_all;
    }
  }
  pz_int_pl->f_hv_protection_level[0] = sqrtf(
    pz_int_pl->f_enu_protection_level[0] * pz_int_pl->f_enu_protection_level[0] +
      pz_int_pl->f_enu_protection_level[1] * pz_int_pl->f_enu_protection_level[1]);
  pz_int_pl->f_hv_protection_level[1] = pz_int_pl->f_enu_protection_level[1];
}

/**
 * @brief Calculate protection level by filter covariance, factor is ppf(1e-5)
 * @param[in/out] pz_pl protection level result
 */
void ppp_IntegCalPlByFilter(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl)
{
  double d_factor = d_ppf[4];  // 1e-5
  // xyz
  pz_pl->f_xyz_protection_level[0] = (float)(sqrt(gpz_int_InfoBlock->d_QEcefPos[0]) * d_factor);
  pz_pl->f_xyz_protection_level[1] = (float)(sqrt(gpz_int_InfoBlock->d_QEcefPos[4]) * d_factor);
  pz_pl->f_xyz_protection_level[2] = (float)(sqrt(gpz_int_InfoBlock->d_QEcefPos[8]) * d_factor);

  // enu
  double* d_QenuPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  if (NULL == d_QenuPos)
  {
    return;
  }
  gnss_CovEcef2Enu(gpz_int_InfoBlock->d_lla, gpz_int_InfoBlock->d_QEcefPos, d_QenuPos);
  pz_pl->f_enu_protection_level[0] = (float)(sqrt(d_QenuPos[0]) * d_factor);
  pz_pl->f_enu_protection_level[1] = (float)(sqrt(d_QenuPos[4]) * d_factor);
  pz_pl->f_enu_protection_level[2] = (float)(sqrt(d_QenuPos[8]) * d_factor);

  // hv
  pz_pl->f_hv_protection_level[0] = sqrtf(pz_pl->f_enu_protection_level[0] * pz_pl->f_enu_protection_level[0] +
    pz_pl->f_enu_protection_level[1] * pz_pl->f_enu_protection_level[1]);
  pz_pl->f_hv_protection_level[1] = pz_pl->f_enu_protection_level[1];

  if (pz_posSolution != NULL)
  {
    pz_posSolution->f_posXyzUnc[0] = pz_pl->f_xyz_protection_level[0];
    pz_posSolution->f_posXyzUnc[1] = pz_pl->f_xyz_protection_level[1];
    pz_posSolution->f_posXyzUnc[2] = pz_pl->f_xyz_protection_level[2];
    pz_posSolution->f_posLlaUnc[0] = pz_pl->f_enu_protection_level[0];
    pz_posSolution->f_posLlaUnc[1] = pz_pl->f_enu_protection_level[1];
    pz_posSolution->f_posLlaUnc[2] = pz_pl->f_enu_protection_level[2];
  }
  OS_FREE(d_QenuPos);
}

/**
 * @brief Get covariance from ekf pool
 * @param[in] p_Q
 * @param[in] pd_index
 * @param[in] n
 * @param[out] p_Qout
 */
static void int_RecoverQ(const double* p_Q, const int16_t* pd_index, uint8_t n, double* p_Qout)
{
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      p_Qout[i * n + j] = p_Q[IUTM(pd_index[i], pd_index[j])];
    }
  }
}

/**
 * @brief Save filter covariance matrix
 * @param[in] pz_PPPfilterInfo float filter info
 * @param[in] pz_fixedAmbPool fix filter info
 * @param[in] pz_EKFstateRepPool ekf state pool
 */
void ppp_IntegSaveFilterPositionMatrix(const ppp_filterInfo_t* pz_PPPfilterInfo,
                                       const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                                       const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  if (!M_IS_SET_BIT(gz_int_Config.u_valid, C_INTEGRITY_LAUNCH_INIT))
  {
    return;
  }
  double* d_QEcefPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  if (any_Ptrs_Null(1, d_QEcefPos))
  {
    OS_FREE(d_QEcefPos);
    return;
  }

  /* P of fixed  */
  if ((NULL != pz_fixedAmbPool) && (GNSS_NONE_AMB_FIXED != pz_fixedAmbPool->u_fixStatus))
  {
    int16_t pd_index[3] = {0, 1, 2};
    int_RecoverQ(pz_fixedAmbPool->pd_q_fix, pd_index, 3, d_QEcefPos);
  }
    /* P of float  */
  else
  {
    int16_t pw_PVAindex[PVA_NUM] = {0};
    getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
    int_RecoverQ(pz_PPPfilterInfo->pd_Q, pw_PVAindex, 3, d_QEcefPos);
  }

  for (int i = 0; i < 9; ++i)
  {
    gpz_int_InfoBlock->d_QEcefPos[i] = d_QEcefPos[i];
  }
  OS_FREE(d_QEcefPos);
}

static void
int_StaticsObsCn0(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_EpochFilterObs_t* pz_filterObs)
{
  uint8_t n = 0;
  float f_mad = 0.0f;
  float* pf_data = OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_TRK_MEAS_NUMBER);
  float* pf_abs = OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_TRK_MEAS_NUMBER);
  if (pf_data == NULL || pf_abs == NULL|| NULL== pz_filterObs)
  {
    OS_FREE(pf_data);
    OS_FREE(pf_abs);
    return;
  }
  for (uint8_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    const gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[i];
    if (pz_satMeas == NULL)
    {
      continue;
    }
    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      gnss_SignalMeas_t* pz_sigMeas = pz_satSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j];
      gnss_MeasStatusFlag_t* pz_sigFlag = pz_filterObs->pz_SatFilterObs[i]->z_measUpdateFlag[j];
      if (pz_sigMeas == NULL || pz_sigFlag == NULL)
      {
        continue;
      }
      // Filter by signal valid
      // if (pz_sigMeas->f_cn0 != 0 && pz_sigFlag->b_valid != 0)
      // {
      //   cn0[n++] = pz_sigMeas->f_cn0;
      // }
      pf_data[n++] = pz_sigMeas->f_cn0;
    }
  }

  /* median */
  gnss_ascSortMedianFloat(pf_data, n, &gpz_pppFeature->f_cn0_avg);

  /* standard */
  gnss_MadFloat(pf_data, n, pf_abs, &f_mad);
  gpz_pppFeature->f_cn0_std = 1.4826f * f_mad;  // std

  OS_FREE(pf_data);
  OS_FREE(pf_abs);
}

static void
int_StaticsObsCmc(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_EpochFilterObs_t* pz_filterObs)
{
  uint8_t n = 0;
  float mad = 0.0f;
  float* pf_data = OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_TRK_MEAS_NUMBER);
  float* pf_abs = OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_TRK_MEAS_NUMBER);
  if (pf_data == NULL || pf_abs == NULL)
  {
    OS_FREE(pf_data);
    OS_FREE(pf_abs);
    return;
  }

  if (pz_filterObs == NULL)
  {
    return;
  }

  for (uint8_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    const gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[i];
    if (pz_satMeas == NULL)
    {
      continue;
    }
    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      gnss_SignalMeas_t* pz_sigMeas = pz_satSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j];
      gnss_MeasStatusFlag_t* pz_sigFlag = pz_filterObs->pz_SatFilterObs[i]->z_measUpdateFlag[j];
      if (pz_sigMeas == NULL || pz_sigFlag == NULL)
      {
        continue;
      }
      /* L-P */
      if ((pz_sigFlag->b_cpValid == 1) && (pz_sigFlag->b_prValid == 1))
      {
        double d_wave = wavelength(pz_sigMeas->u_signal);
        float delta = (float)(pz_sigMeas->d_pseudoRange - pz_sigMeas->d_carrierPhase * d_wave);
        delta = fabsf(delta);
        pf_data[n++] = delta;
      }
    }
  }
  /* Median */
  gnss_ascSortMedianFloat(pf_data, n, &gpz_pppFeature->f_cmc_avg);

  /* STD by MAD */
  gnss_MadFloat(pf_data, n, pf_abs, &mad);
  gpz_pppFeature->f_cmc_std = 1.4826f * mad;  // std

  OS_FREE(pf_data);
  OS_FREE(pf_abs);
}

static void int_StaticsObsPrVar(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t n = 0;
  float* pf_data = OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_TRK_MEAS_NUMBER);
  if (pf_data == NULL)
  {
    return;
  }

  for (uint8_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    const gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[i];
    if (pz_satMeas == NULL)
    {
      continue;
    }
    for (uint8_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      gnss_SignalMeas_t* pz_sigMeas = pz_satSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j];
      if (pz_sigMeas == NULL)
      {
        continue;
      }
      if (pz_sigMeas->d_pr_unc != 0)
      {
        pf_data[n++] = (float)sqrt(pz_sigMeas->d_pr_unc);
      }
    }
  }
  gpz_pppFeature->f_avg_unc_pr = gnss_MeanFloat(pf_data, n);
  OS_FREE(pf_data);
}

/**
 * @brief pseudo-range residual TODO(houxiaowei): P-empirical value
 * @param pz_satSigMeasCollect
 */
static void int_StaticsPrRes(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  if (gnss_Norm(gpz_int_InfoBlock->d_QEcefPos, 3) <= 0)
  {
    LOGI(TAG_PPP, "integrity fill pr res failed, pos is empty\n")
    return;
  }
  // r, dts, tgd

}

/**
 * @brief Machine Learning DLOG_GNSS_PL
 */
void log_MachineLearingData()
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  if (gpz_pppFeature->u_fixFlag == GNSS_FIX_FLAG_SPS ||
    gpz_pppFeature->u_fixFlag == GNSS_FIX_FLAG_INVALID)
  {
    return;
  }

  const GpsTime_t* gpst = &gpz_pppFeature->z_tor;
  EpochTime_t z_epochT = {0};
  tm_cvt_GpstToEpoch(gpst, &z_epochT);

  DATA_LOG_HEADER(DLOG_GNSS_ML_PL, "week, tow, utc_time, state, u_fixStatus, "//line 1:5
    "f_pdop, f_hdop, f_vdop, f_gdop, f_ambPool_adop, f_ambPool_pdop, u_CN040, f_age, "//line 2:8
    "f_cn0_avg, f_cn0_std, f_cmc_avg, f_cmc_std, f_avg_unc_pr, pl_ekf_hor, pl_ekf_ver, "//line 3:7
    "z_sceneType, openSkyCount, closeSkyCount, f_ratio, u_ns, u_nb, "//line 4:6
    "u_satMeasCount, u_fixedTotalSatCount, u_ppp_use_cp_num_float, u_ppp_use_pr_num_float, u_ppp_use_sat_num_fix, "//line 5:5
    "u_MeasTrackCount, u_MeasInUseCount, u_SvTrackCount, u_SvInUseCount, q_QRcheckStatus, "//line 6:5
    "d_lambda_sigma_1, d_lambda_sigma_2, "//line 7:2
    "f_postCodeSTD, f_postPhaseSTD, f_fixedCodeSTD, f_fixedPhaseSTD, "//line 8:4
    "\n"
  );
  // $week, sec, utc_time, flag, use_sat_num_float, pdop, cno_avg, cno_std, cmc_avg, cmc_std, avg_res_pr, avg_unc_pr, pl_hor, pl_ver
  DATA_LOG(DLOG_GNSS_ML_PL,
    "%d, %.1f, %4d/%02d/%02d %02d:%02d:%05.2f, %d, %d, "//line 1:5
    "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %.3f, "//line 2:8
    "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, "//line 3:7
    "%d, %d, %d, %.3f, %d, %d, "//line 4:6
    "%d, %d, %d, %d, %d, "//line 5:5
    "%d, %d, %d, %d, %d, "//line 6:5
    "%.3f, %.3f, "//line 7:2
    "%.3f, %.3f, %.3f, %.3f, "//line 8:4
    "\n",
    gpst->w_week, gpst->q_towMsec * TIME_MSEC_INV, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour,
    z_epochT.min, z_epochT.second,
    gpz_pppFeature->u_fixFlag, gpz_pppFeature->u_fixStatus, //line 1:5
    gpz_pppFeature->z_dops.f_pdop, gpz_pppFeature->z_dops.f_hdop, gpz_pppFeature->z_dops.f_vdop,
    gpz_pppFeature->z_dops.f_gdop,
    gpz_pppFeature->f_ambPool_adop, gpz_pppFeature->f_ambPool_pdop, gpz_pppFeature->u_CN040,
    gpz_pppFeature->f_age, //line 2:8
    gpz_pppFeature->f_cn0_avg, gpz_pppFeature->f_cn0_std, gpz_pppFeature->f_cmc_avg, gpz_pppFeature->f_cmc_std,
    gpz_pppFeature->f_avg_unc_pr,
    gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0],
    gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1], //line 3:7
    gpz_pppFeature->z_sceneType, ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount),
    ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount),
    gpz_pppFeature->f_ratio, gpz_pppFeature->u_ns, gpz_pppFeature->u_nb, //line 4:6
    gpz_pppFeature->u_satMeasCount, gpz_pppFeature->u_fixedTotalSatCount, gpz_pppFeature->u_ppp_use_cp_num_float,
    gpz_pppFeature->u_ppp_use_pr_num_float, gpz_pppFeature->u_ppp_use_sat_num_fix, //line 5:5
    gpz_pppFeature->z_SvStatus.u_MeasTrackCount, gpz_pppFeature->z_SvStatus.u_MeasInUseCount,
    gpz_pppFeature->z_SvStatus.u_SvTrackCount,
    gpz_pppFeature->z_SvStatus.u_SvInUseCount, gpz_pppFeature->q_QRcheckStatus, //line 6:5
    gpz_pppFeature->d_lambda_sigma[0], gpz_pppFeature->d_lambda_sigma[1], //line 7:2
    gpz_pppFeature->f_postCodeSTD, gpz_pppFeature->f_postPhaseSTD, gpz_pppFeature->f_fixedCodeSTD,
    gpz_pppFeature->f_fixedPhaseSTD //line 8:4
  );

  DATA_LOG_HEADER(DLOG_GNSS_ML_SCENE, "week, tow, utc_time, "//line 1:3
                                      "f_pdop, f_hdop, f_vdop, f_gdop, u_CN040, "//line 2:5
                                      "f_cn0_avg, f_cn0_std, f_cmc_avg, f_cmc_std, f_avg_unc_pr, pl_ekf_hor, pl_ekf_ver, "//line 3:7
                                      "u_ns, u_satMeasCount, u_ppp_use_cp_num_float, u_ppp_use_pr_num_float, u_MeasTrackCount, u_MeasInUseCount, "//line 4:6
                                      "q_QRcheckStatus, f_postCodeSTD, f_postPhaseSTD, f_posLlaUnc_b, f_posLlaUnc_l, f_posLlaUnc_h, "//line 5:6
                                      "w_cn0Num, w_LLInum, w_noLLInum, f_max_CN0, pu_LLIsatNum_G, pu_LLIsatNum_C, "//line 6:6
                                      "pu_noLLIsatNum_G, pu_noLLIsatNum_C, pu_satNumPerSys_G, pu_satNumPerSys_C, "//line 7:4
                                      "pu_CN0greater35NumPerSys_G, pu_CN0greater35NumPerSys_C, u_sysNum"//line 8:3
                                      "\n"
  );
  DATA_LOG(DLOG_GNSS_ML_SCENE,
           "%d, %.1f, %4d/%02d/%02d %02d:%02d:%05.2f, "//line 1:3
           "%.3f, %.3f, %.3f, %.3f, %d, "//line 2:5
           "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, "//line 3:7
           "%d, %d, %d, %d, %d, %d, "//line 4:6
           "%d, %.3f, %.3f, %.3f, %.3f, %.3f, "//line 5:6
           "%d, %d, %d, %.3f, %d, %d, "//line 6:6
           "%d, %d, %d, %d, "//line 7:4
           "%d, %d, %d"//line 8:3
           "\n",
           gpst->w_week, gpst->q_towMsec * TIME_MSEC_INV, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour,
           z_epochT.min, z_epochT.second, //line 1:3
           gpz_pppFeature->z_dops.f_pdop, gpz_pppFeature->z_dops.f_hdop, gpz_pppFeature->z_dops.f_vdop,
           gpz_pppFeature->z_dops.f_gdop, gpz_pppFeature->u_CN040, //line 2:5
           gpz_pppFeature->f_cn0_avg, gpz_pppFeature->f_cn0_std, gpz_pppFeature->f_cmc_avg, gpz_pppFeature->f_cmc_std,
           gpz_pppFeature->f_avg_unc_pr,
           gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0],
           gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1], //line 3:7
           gpz_pppFeature->u_ns, gpz_pppFeature->u_satMeasCount, gpz_pppFeature->u_ppp_use_cp_num_float,
           gpz_pppFeature->u_ppp_use_pr_num_float,
           gpz_pppFeature->z_SvStatus.u_MeasTrackCount, gpz_pppFeature->z_SvStatus.u_MeasInUseCount, //line 4:6
           gpz_pppFeature->q_QRcheckStatus, gpz_pppFeature->f_postCodeSTD, gpz_pppFeature->f_postPhaseSTD,
           gpz_pppFeature->f_posLlaUnc[0],
           gpz_pppFeature->f_posLlaUnc[1], gpz_pppFeature->f_posLlaUnc[2], //line 5:6
           gpz_pppFeature->w_cn0Num, gpz_pppFeature->w_LLInum, gpz_pppFeature->w_noLLInum, gpz_pppFeature->f_max_CN0,
           gpz_pppFeature->pu_LLIsatNum[C_GNSS_GPS],
           gpz_pppFeature->pu_LLIsatNum[C_GNSS_BDS3], //line 6:6
           gpz_pppFeature->pu_noLLIsatNum[C_GNSS_GPS], gpz_pppFeature->pu_noLLIsatNum[C_GNSS_BDS3],
           gpz_pppFeature->pu_satNumPerSys[C_GNSS_GPS],
           gpz_pppFeature->pu_satNumPerSys[C_GNSS_BDS3], //line 7:4
           gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_GPS], gpz_pppFeature->pu_CN0greater35NumPerSys[C_GNSS_BDS3],
           gpz_pppFeature->u_sysNum); //line 8:3
}

/**
 * @brief get the num of bit 1 from an uint64_t integer
 * @param[in] uint64_t integer
 */
uint8_t ppp_IntegCountSetBits(uint64_t t_n)
{
  uint8_t u_count = 0;
  while (t_n)
  {
    t_n &= (t_n - 1);
    u_count++;
  }
  return u_count;
}

/**
 *@brief add the sky scene to the pppFeature
 * @param[in] pz_filterInfo
 */
void ppp_IntegSetScene(const ppp_filterInfo_t* pz_filterInfo)
{
  if ((pz_filterInfo->t_openSkyCount & 0x01) == 0x01)
  {
    gpz_pppFeature->z_sceneType = OPEN_SKY_SCENE;
  }
  else if ((pz_filterInfo->t_closeSkyCount & 0x01)== 0x01)
  {
    gpz_pppFeature->z_sceneType = CLOSE_SKY_SCENE;
  }
  else
  {
    gpz_pppFeature->z_sceneType = SEMI_SKY_SCENE;
  }
  gpz_pppFeature->z_sceneType = pz_filterInfo->z_sceneType;
  gpz_pppFeature->t_openSkyCount = pz_filterInfo->t_openSkyCount;
  gpz_pppFeature->t_closeSkyCount = pz_filterInfo->t_closeSkyCount;
}

/**
 *@brief add feature from pz_satSigMeasCollect for the scene determined by machine learning
 * @param[in] pz_satSigMeasCollect
 */
void ppp_IntegSetMLScene(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint16_t w_satNum = 0;
  uint16_t w_cn0Num = 0;
  uint16_t w_LLInum = 0;
  uint16_t w_noLLInum = 0;
  uint8_t u_signalType = 0;
  uint8_t u_signalIndex = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_L1;
  uint32_t q_satIndex = 0;
  uint32_t u_svIdx = 0;
  uint8_t u_constellation = 0;
  uint8_t u_constellationSceneUsed = 0;
  uint8_t u_svid = 0;
  uint8_t u_sysNum = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_SignalMeas_t* pz_L1SigMeas = NULL;
  float f_maxCN0 = 0.0;
  uint8_t pu_LLIsatNum[C_GNSS_MAX] = {0};
  uint8_t pu_noLLIsatNum[C_GNSS_MAX] = {0};
  uint8_t pu_satNumPerSys[C_GNSS_MAX] = {0};
  uint8_t pu_CN0greater35NumPerSys[C_GNSS_MAX] = {0};
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = (const gnss_SatelliteMeas_t*)(pz_satSigMeasCollect->pz_satMeas[q_satIndex]);
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = pz_satMeas->u_constellation;
    u_constellationSceneUsed = u_constellation;
    if (C_GNSS_BDS2 == u_constellation || C_GNSS_BDS3 == u_constellation)
    {
      u_constellationSceneUsed = C_GNSS_BDS3;
    }
    u_svid = pz_satMeas->u_svid;
    u_svIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
    if (ALL_GNSS_SYS_SV_NUMBER == u_svIdx)
    {
      continue;
    }
    pz_L1SigMeas = NULL;
    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = (const gnss_SignalMeas_t*)(pz_satMeas->pz_signalMeas[u_signalIndex]);
      if (NULL == pz_sigMeas || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      u_signalType = pz_sigMeas->u_signal;
      u_freqType = gnss_cvt_Sig2FreqType(u_signalType);
      if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
      {
        pz_L1SigMeas = pz_sigMeas;
        break;
      }
    }
    if (NULL == pz_L1SigMeas)
    {
      continue;
    }
    ++pu_satNumPerSys[u_constellationSceneUsed];
    ++w_satNum;
    //start for CN0
    if ((pz_L1SigMeas->f_cn0) > 0.0)
    {
      ++w_cn0Num;
      if ((pz_L1SigMeas->f_cn0) > f_maxCN0)
      {
        f_maxCN0 = (pz_L1SigMeas->f_cn0);
      }
      if ((pz_L1SigMeas->f_cn0) >= 35.0)
      {
        ++pu_CN0greater35NumPerSys[u_constellationSceneUsed];
      }
    }
    //end for CN0

    //start for LLI
    if ((pz_L1SigMeas->u_LLI) > 0)
    {
      ++pu_LLIsatNum[u_constellationSceneUsed];
      ++w_LLInum;
    }
    else
    {
      ++pu_noLLIsatNum[u_constellationSceneUsed];
      ++w_noLLInum;
    }
    //end for LLI
  }

  //calculating the number of constellation
  for (u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; ++u_constellation)
  {
    if (pu_satNumPerSys[u_constellation] > 0)
    {
      ++u_sysNum;
    }
  }

  gpz_pppFeature->w_cn0Num = w_cn0Num;
  gpz_pppFeature->w_LLInum = w_LLInum;
  gpz_pppFeature->w_noLLInum = w_noLLInum;
  gpz_pppFeature->f_max_CN0 = f_maxCN0;
  gpz_pppFeature->u_sysNum = u_sysNum;
  for (uint8_t u_i = 0; u_i < C_GNSS_MAX; u_i++)
  {
    gpz_pppFeature->pu_LLIsatNum[u_i] = pu_LLIsatNum[u_i];
    gpz_pppFeature->pu_noLLIsatNum[u_i] = pu_noLLIsatNum[u_i];
    gpz_pppFeature->pu_satNumPerSys[u_i] = pu_satNumPerSys[u_i];
    gpz_pppFeature->pu_CN0greater35NumPerSys[u_i] = pu_CN0greater35NumPerSys[u_i];
  }
}

/**
 *@brief add the feature from gz_pppAmbFixInputInfo/pz_fixedAmbPool/pz_PositionFix to the pppFeature
 * @param[in] pz_pppAmbFixInputInfo
 * @param[in] pz_fixedAmbPool
 * @param[in] pz_PositionFix
 */
void ppp_IntegSetOtherFeature(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
                              const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                              const gnss_PositionFix_t* pz_PositionFix)
{
  /* feature from pz_PostionFix */
  //gpz_pppFeature->u_CN040 = pz_PositionFix->u_CN040;
  for (uint8_t u_i = 0; u_i < 3; u_i++)
  {
    gpz_pppFeature->f_velEnu[u_i] = fabsf(pz_PositionFix->f_velEnu[u_i]);
    gpz_pppFeature->f_posLlaUnc[u_i] = pz_PositionFix->f_posLlaUnc[u_i];
  }
  gpz_pppFeature->z_SvStatus = pz_PositionFix->z_SvStatus;

  /* feature from pz_fixedAmbPool */
  gpz_pppFeature->u_fixStatus = pz_fixedAmbPool->u_fixStatus;
  for (uint8_t u_i = 0; u_i < C_GNSS_MAX; u_i++)
  {
    gpz_pppFeature->u_fixedns[u_i] = pz_fixedAmbPool->u_fixedns[u_i];
  }
  gpz_pppFeature->u_ppp_use_sat_num_wideLane = pz_fixedAmbPool->u_ppp_use_sat_num_wideLane;
  gpz_pppFeature->u_ppp_use_sat_num_fix = pz_fixedAmbPool->u_ppp_use_sat_num_fix;
  gpz_pppFeature->u_fixedTotalSatCount = pz_fixedAmbPool->u_fixedTotalSatCount;
  gpz_pppFeature->u_nb = pz_fixedAmbPool->u_nb;
  gpz_pppFeature->f_ratio = pz_fixedAmbPool->f_ratio;
  for (uint8_t u_i = 0; u_i < 2; u_i++)
  {
    gpz_pppFeature->d_lambda_sigma[u_i] = pz_fixedAmbPool->f_lambda_sigma[u_i];
  }
  gpz_pppFeature->f_ambPool_adop = pz_fixedAmbPool->f_adop;
  gpz_pppFeature->f_ambPool_pdop = pz_fixedAmbPool->f_pdop;

  /* feature from pz_pppAmbFixInputInfo */
  const gnss_SatSigMeasCollect_t* pz_satSignalPool = pz_pppAmbFixInputInfo->pz_satSignalPool;
  gpz_pppFeature->u_satMeasCount = pz_satSignalPool->u_satMeasCount;

  ppp_filterInfo_t* pz_pppFilterInfo = pz_pppAmbFixInputInfo->pz_pppFilterInfo;
  gpz_pppFeature->u_ns = pz_pppFilterInfo->u_ns;
  gpz_pppFeature->f_age = pz_pppFilterInfo->f_age;
  gpz_pppFeature->q_QRcheckStatus = pz_pppFilterInfo->q_QRcheckStatus;

  ppp_EpochFilterObs_t* pz_FilterObs = pz_pppAmbFixInputInfo->pz_FilterObs;
  gpz_pppFeature->u_ppp_use_dr_num_float = pz_FilterObs->u_ppp_use_dr_num_float;
  gpz_pppFeature->u_ppp_use_cp_num_float = pz_FilterObs->u_ppp_use_cp_num_float;
  gpz_pppFeature->u_ppp_use_pr_num_float = pz_FilterObs->u_ppp_use_pr_num_float;
  gpz_pppFeature->f_postCodeSTD = pz_FilterObs->f_postCodeSTD;
  /*if (gpz_pppFeature->f_postCodeSTD < 1.0) gpz_pppFeature->f_postCodeSTD += 1.5;
  if (gpz_pppFeature->f_postCodeSTD < 1.5) gpz_pppFeature->f_postCodeSTD += 1.0;*/
  gpz_pppFeature->f_postPhaseSTD = pz_FilterObs->f_postPhaseSTD;
  gpz_pppFeature->f_fixedCodeSTD = pz_FilterObs->f_fixedCodeSTD;
  gpz_pppFeature->f_fixedPhaseSTD = pz_FilterObs->f_fixedPhaseSTD;

  //TODO: output the features in SSR data
  // gnss_ssrLosBlock_t* pz_ssrLocBlk = pz_pppAmbFixInputInfo->pz_ssrLocBlk;
}

/**
 * @brief Fill up Machine Learning date feature information
 * @param[in] pz_satSigMeasCollect satellite observation
 * @param[in] pz_PositionFix position info
 * @param[in] pz_filterInfo float filter info
 * @param[in] pz_EKFstateRepPool EKF state pool
 * @param[in] pz_filterObs  information of observation after filter
 */
void
ppp_IntegFillMlFeature(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_PositionFix_t* pz_PositionFix,
                       const ppp_filterInfo_t* pz_filterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
                       const ppp_EpochFilterObs_t* pz_filterObs, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
                       const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  gpz_pppFeature->z_tor = pz_satSigMeasCollect->z_tor;

  // gpz_pppFeature->z_dops = pz_PositionFix->z_dops;
  gpz_pppFeature->z_dops = gpz_int_InfoBlock->z_dops;
  LOGI(TAG_PPP, "delta pdop %f\n", gpz_int_InfoBlock->z_dops.f_pdop - pz_PositionFix->z_dops.f_pdop);

  ppp_IntegSetScene(pz_filterInfo);

  ppp_IntegSetOtherFeature(pz_pppAmbFixInputInfo, pz_fixedAmbPool, pz_PositionFix);

  /* Filter protection level */
  ppp_IntegCalPlByFilter(NULL, &gpz_pppFeature->z_pl_ekf_result);

  /* Cn0 */
  int_StaticsObsCn0(pz_satSigMeasCollect, pz_filterObs);

  /* Code-minus-carrier */
  int_StaticsObsCmc(pz_satSigMeasCollect, pz_filterObs);

  /* Observation pseudo-range variance */
  int_StaticsObsPrVar(pz_satSigMeasCollect);
}

static void integ_unc_smt_op(integ_Uncsmt_t* uncsmt, double hvunc, int state)
{
  int i = 0;
  /*smt detect*/
  int smtext = 1;
  double ssum = 0;
  double sum = 0;
  int qnum = 4;

  //push unc in buffer
  if (uncsmt->uncnum < MAXUNCBUFF)
  {
    for (i = uncsmt->uncnum - 1; i > 0; --i)
    {
      uncsmt->unc_buff[i] = uncsmt->unc_buff[i - 1];
      uncsmt->state_buff[i] = uncsmt->state_buff[i - 1];
    }
    uncsmt->unc_buff[0] = hvunc;
    uncsmt->state_buff[0] = state;
    uncsmt->uncnum++;
  }
  else
  {
    for (i = MAXUNCBUFF - 1; i > 0; --i)
    {
      uncsmt->unc_buff[i] = uncsmt->unc_buff[i - 1];
      uncsmt->state_buff[i] = uncsmt->state_buff[i - 1];
    }
    uncsmt->unc_buff[0] = hvunc;
    uncsmt->state_buff[0] = state;
  }
  //smt start
  if (state == GNSS_FIX_FLAG_FIXED && uncsmt->smting != 1)
  {
    if (fabs(hvunc) > SMT_FIXUNCHIGHB) // SMT_FIXUNCHIGHB = 1.2
    {
      uncsmt->smting = 1;
      uncsmt->unc0 = hvunc;
    }
    uncsmt->smtlen = 0;
    uncsmt->smtlen0 = 0;
  }
  else if ((state == GNSS_FIX_FLAG_FLOATING) && uncsmt->smting != 1)
  {
    if (fabs(hvunc) > SMT_FLTUNCHIGHB) // SMT_FLTUNCHIGHB = 7.0
    {
      uncsmt->smting = 1;
      uncsmt->unc0 = hvunc;
    }
    uncsmt->smtlen = 0;
    uncsmt->smtlen0 = 0;
  }
  else if ((state == GNSS_FIX_FLAG_SPS) && uncsmt->smting != 1)
  {
    if (fabs(hvunc) > SMT_SPPUNCHIGHB) // SMT_SPPUNCHIGHB = 10.0
    {
      uncsmt->smting = 1;
      uncsmt->unc0 = hvunc;
    }
    uncsmt->smtlen = 0;
    uncsmt->smtlen0 = 0;
  }
  else
  { ;
  }
  //quit detect
  if (uncsmt->smting == 1 && uncsmt->uncnum >= qnum) // qnum = 4
  {
    for (i = 0; i < qnum; i++)
    {
      if (uncsmt->state_buff[i] == GNSS_FIX_FLAG_FIXED)
      {
        if (uncsmt->unc_buff[i] < SMT_FIXUNCLOWB) smtext &= 1;
        else smtext &= 0;
      }
      else if (uncsmt->state_buff[i] == GNSS_FIX_FLAG_FLOATING)
      {
        if (uncsmt->unc_buff[i] < SMT_FLTUNCLOWB) smtext &= 1;
        else smtext &= 0;
      }
      else if (uncsmt->state_buff[i] == GNSS_FIX_FLAG_SPS)
      {
        if (uncsmt->unc_buff[i] < SMT_SPPUNCLOWB) smtext &= 1;
        else smtext &= 0;
      }
      else
      { ;
      }
    }
  }
  else smtext &= 0;
  if (smtext == 1 && uncsmt->smting == 1)
  {
    uncsmt->smting = 0;
    uncsmt->smtlen = 0;
    uncsmt->smtlen0 = 0;
  }

  /*do smt*/
  if (uncsmt->smting == 1)
  {
    //window update
    //if (hvunc / uncsmt->unc0 > 1.5 || hvunc - uncsmt->unc0 > 3.0)
    if (hvunc / uncsmt->unc0 > 1.5)
    {
      uncsmt->unc0 = hvunc;
      uncsmt->smtlen = 0;
    }
    if (uncsmt->smtlen < MAXUNCBUFF)
    {
      uncsmt->smtlen++;
      uncsmt->smtlen0++;
    }
    ssum = gnss_Dot(uncsmt->unc_buff, uncsmt->unc_buff, uncsmt->smtlen);
    for (i = 0; i < uncsmt->smtlen; i++)
    {
      sum += uncsmt->unc_buff[i];
    }
    uncsmt->unc_smt = ssum / sum;
  }
  else
  {
    uncsmt->unc_smt = hvunc;
  }
}

void ppp_IntegSetPLResult(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl) {
  /* for now, gpz_pppFeature->u_fixFlag = 0 means single point positioning. */
  if (gpz_pppFeature->u_fixFlag == GNSS_FIX_FLAG_INVALID && pz_posSolution->u_fixFlag == GNSS_FIX_FLAG_SPS)
  {
    pz_pl->u_fixFlagType = GNSS_FIX_FLAG_SPS;
  }
  else
  {
    pz_pl->u_fixFlagType = gpz_pppFeature->u_fixFlag;
  }
}

double ppp_IntegExtraRulesForFloatPL(uint8_t u_scene, double d_pl_hor) {
  // extra rules
  double d_result = d_pl_hor;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  switch (u_scene)
  {
  case 1:
    d_result += 0;
    break;
  case 2:
    d_result += 1;
    break;
  case 3:
    d_result += 2.5;
    break;
  case 4:
    d_result += 3;
    break;
  }
  if (pl_ekf_hor >= 0.83 && pl_ekf_hor <= 0.90 && d_result >= 2.2 && d_result <= 2.25)
  {
    d_result += 0.1;
  }
  if (gpz_pppFeature->u_CN040 <= 12 && u_scene == 1)
  {
    d_result += 0.5;
  }
  return d_result;
}

double ppp_IntegExtraRulesForFixedPL(uint8_t u_scene, double d_pl_hor) {
  // extra rules
  double d_result = d_pl_hor;
  switch (u_scene)
  {
  case 1:
    d_result += 0;
    break;
  case 2:
    d_result += 1.0;
    break;
  case 3:
    d_result += 2;
    break;
  case 4:
    d_result += 2.5;
    break;
  }
  if (gpz_pppFeature->f_ratio <= 1.2 && gpz_pppFeature->f_postCodeSTD >= 1.5 && gpz_pppFeature->f_postCodeSTD <= 2.9)
  {
    d_result += 3;
  }
  if (gpz_pppFeature->f_ratio >= 2.4 && gpz_pppFeature->f_ratio <= 4.5 && gpz_pppFeature->f_postCodeSTD >= 2.0 &&
    gpz_pppFeature->f_postCodeSTD <= 2.5)
  {
    d_result += 3;
  }
  if (gpz_pppFeature->f_ratio >= 1.35 && gpz_pppFeature->f_ratio <= 1.40 && gpz_pppFeature->f_postCodeSTD >= 1.23 &&
    gpz_pppFeature->f_postCodeSTD <= 1.3)
  {
    d_result += 1;
  }
  if (gpz_pppFeature->f_ratio >= 1.55 && gpz_pppFeature->f_ratio <= 1.60 && gpz_pppFeature->f_postCodeSTD >= 0.95 &&
    gpz_pppFeature->f_postCodeSTD <= 1.0)
  {
    d_result += 0.5;
  }
  if (gpz_pppFeature->z_dops.f_pdop > 2.7 && u_scene == 3)
  {
    d_result += 2.0;
  }
  if (gpz_pppFeature->z_dops.f_pdop >= 1.84 && gpz_pppFeature->z_dops.f_pdop <= 1.90 &&
    gpz_pppFeature->u_CN040 <= 18 && u_scene == 2)
  {
    d_result += 1;
  }
  if (gpz_pppFeature->u_CN040 <= 5)
  {
    d_result += 1.5;
  }
  if (gpz_pppFeature->z_dops.f_pdop >= 2.1 && gpz_pppFeature->z_dops.f_pdop <= 2.15 &&
    gpz_pppFeature->f_postCodeSTD >= 2.0 && gpz_pppFeature->f_postCodeSTD <= 2.1)
  {
    d_result += 1;
  }
  return d_result;
}

void ppp_IntegCalPLByDecisionTree(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl)
{
  double d_pl_hor = 0.0;
  double d_pl_ver = 0.0;
  memset(pz_pl, 0, sizeof(integ_PLResult_t));

  ppp_IntegSetPLResult(pz_posSolution, pz_pl);

  uint8_t u_scene = integ_DecisionTreePredictScene(gpz_pppFeature);
  /* extra rules for scene */
  if (gpz_pppFeature->z_sceneType != OPEN_SKY_SCENE)
  {
    u_scene = 4;
  }
  pz_pl->u_sceneType = u_scene;

  /* Call Decision Tree model */
  switch (pz_pl->u_fixFlagType)
  {
  case GNSS_FIX_FLAG_INVALID:
    break;
  case GNSS_FIX_FLAG_SPS:
    /* PL for single point positioning */
    pz_pl->f_enu_protection_level[0] = pz_posSolution->f_posLlaUnc[0] * 3;
    pz_pl->f_enu_protection_level[1] = pz_posSolution->f_posLlaUnc[1] * 3;
    pz_pl->f_enu_protection_level[2] = pz_posSolution->f_posLlaUnc[2] * 3;

    // hv
    pz_pl->f_hv_protection_level[0] = sqrtf(pz_pl->f_enu_protection_level[0] * pz_pl->f_enu_protection_level[0] +
      pz_pl->f_enu_protection_level[1] * pz_pl->f_enu_protection_level[1]);
    pz_pl->f_hv_protection_level[1] = pz_pl->f_enu_protection_level[1];
    d_pl_hor = pz_pl->f_hv_protection_level[0];
    break;
  case GNSS_FIX_FLAG_FLOATING:
    d_pl_hor = integ_DecisionTreePredictHorFloat(gpz_pppFeature);
    /* extral rules*/
    d_pl_hor = ppp_IntegExtraRulesForFloatPL(u_scene, d_pl_hor);
    break;
  case GNSS_FIX_FLAG_FIXED:
    d_pl_hor = integ_DecisionTreePredictHorFix(gpz_pppFeature);
    /* extral rules*/
    d_pl_hor = ppp_IntegExtraRulesForFixedPL(u_scene, d_pl_hor);
    break;
  }

  /* window select */
  integ_unc_smt_op(&ppp_uncsmt, d_pl_hor, pz_pl->u_fixFlagType);

  /* Horizontal / vertical/ ENU protection level */
  pz_pl->f_hv_protection_level[0] = (float)(ppp_uncsmt.unc_smt * 2.0);
  pz_pl->f_hv_protection_level[1] = (float)(ppp_uncsmt.unc_smt * 2.0 * 2.0);
  pz_pl->f_enu_protection_level[0] = (float)(pz_pl->f_hv_protection_level[0] / 1.414);
  pz_pl->f_enu_protection_level[1] = (float)(pz_pl->f_hv_protection_level[0] / 1.414);
  pz_pl->f_enu_protection_level[2] = (float)pz_pl->f_hv_protection_level[1];

  /* integrity protection level and flag */
  pz_posSolution->z_integrity.z_pos_hor_pl = gnss_genProtectionLevelStruct(pz_pl->f_hv_protection_level[0]);
  pz_posSolution->z_integrity.z_pos_ver_pl = gnss_genProtectionLevelStruct(pz_pl->f_hv_protection_level[1]);
  pz_posSolution->z_integrity.z_pos_north_pl = gnss_genProtectionLevelStruct(pz_pl->f_enu_protection_level[1]);
  pz_posSolution->z_integrity.z_pos_east_pl = gnss_genProtectionLevelStruct(pz_pl->f_enu_protection_level[0]);

}