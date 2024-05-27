#include "ppp_process.h"
#include "gnss_def.h"
#include "ppp_filter_sol.h"
#include "gnss_engine_api.h"
#include "gnss_tdcp_detectCS.h"
#include "gnss_prep.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "ppp_amb_fix.h"
#include "cmn_utils.h"
#include "gnss_common.h"
#include "cmn_turboEdit.h"
#include "qxsi_ids_ssr2los_interface.h"
#include "sm_api.h"
#include "sd_api.h"
#include "sm_nmea.h"
#include "ppp_task.h"
#include "gnss_identify_scene.h"
#include "cmn_dopplerDetectCS.h"
#include "ppp_obsnoise.h"
#include "gnss_identify_scene.h"
#include <math.h>
#include "ppp_integrity.h"
#include "ppp_qrcheck.h"
#include "windup_correction.h"
#include "tide_correction.h"
#include "ppp_los_predict.h"

ppp_filterInfo_t gz_PPPfilterInfo;
gnss_EKFstateRepresentPool_t gz_EKFstateRepPool;
gnss_pppAmbFixInputInfo_t gz_pppAmbFixInputInfo;
cmn_turboEditFilterSet gz_gfDetectFilter;
cmn_turboEditFilterSet gz_MwDetectFilter;
gnss_dopplerDetectPairingBlock_t gz_dopplerDetectPairingBlock;
gnss_TdcpMeasBlock_t* gpz_preTdcpMeasPPP = NULL;
ppp_LosPredictBlock_t* gpz_losPredictBlock;

/**
*@brief load the default option for the PPP algorithm
* @param[in]  pz_opt represent the algorithm optimization for the PPP-RTK
* @param[out] void
* @return     void
*/
void PPP_loadDefaultOption(ppp_alg_opt_t* pz_opt)
{
  pz_opt->z_usedSys = (ALGO_GPS_SYS | ALGO_BDS_SYS | ALGO_GLO_SYS | ALGO_GAL_SYS);
  pz_opt->z_usedFreq = (ALGO_L1_FREQ | ALGO_L2_FREQ | ALGO_L5_FREQ);
  pz_opt->q_isSperateBDS2And3 = TRUE;
  pz_opt->d_elmin = 7.0 * DEG2RAD;
  return;
}

/**
 * @brief malloc pz_ssrStatus
 * @return     ppp_ssrEpochCorrStatus_t pointer
 */
static ppp_EpochFilterObs_t* malloc_pz_ssrStatus(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_status = 0;
  ppp_EpochFilterObs_t* pz_filterObs=NULL;

  pz_filterObs = (ppp_EpochFilterObs_t*)OS_MALLOC_FAST(sizeof(ppp_EpochFilterObs_t));
  if (NULL == pz_filterObs)
  {
    return NULL;
  }
  
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == pz_satSigMeasCollect->pz_satMeas[u_i])
    {
      pz_filterObs->pz_SatFilterObs[u_i] = NULL;
      continue;
    }
    pz_filterObs->pz_SatFilterObs[u_i] = (ppp_SatFilterObs_t*)OS_MALLOC_FAST(sizeof(ppp_SatFilterObs_t));
    if (NULL == pz_filterObs->pz_SatFilterObs[u_i])
    {
      LOGW(TAG_PPP, "%s pz_ssrSatCorrStatus malloc failed", __FUNCTION__);
      u_status = 1;
      break;
    }
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      if (NULL == pz_satSigMeasCollect->pz_satMeas[u_i]->pz_signalMeas[u_j])
      {
        pz_filterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_codeResidual[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_codeVar[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseVar[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_codeSigma[u_j] = NULL;
        pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseSigma[u_j] = NULL;
        continue;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j]
        = (ppp_ssrSignalBiasStatus_t*)OS_MALLOC_FAST(sizeof(ppp_ssrSignalBiasStatus_t));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j])
      {
        LOGW(TAG_PPP, "%s pz_signalBiasCorrStatus malloc failed", __FUNCTION__);
        u_status = 2;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j]
        = (gnss_MeasStatusFlag_t*)OS_MALLOC_FAST(sizeof(gnss_MeasStatusFlag_t));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j])
      {
        LOGW(TAG_PPP, "%s z_measStatusFlag malloc failed", __FUNCTION__);
        u_status = 3;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_codeResidual[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_codeResidual[u_j])
      {
        LOGW(TAG_PPP, "%s pf_codeResidual malloc failed", __FUNCTION__);
        u_status = 4;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j])
      {
        LOGW(TAG_PPP, "%s pf_phaseResidual malloc failed", __FUNCTION__);
        u_status = 5;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_codeVar[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_codeVar[u_j])
      {
        LOGW(TAG_PPP, "%s pf_codeVar malloc failed", __FUNCTION__);
        u_status = 6;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseVar[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseVar[u_j])
      {
        LOGW(TAG_PPP, "%s pf_phaseVar malloc failed", __FUNCTION__);
        u_status = 7;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_codeSigma[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_codeSigma[u_j])
      {
        LOGW(TAG_PPP, "%s pf_codeSigma malloc failed", __FUNCTION__);
        u_status = 8;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseSigma[u_j]
        = (float*)OS_MALLOC_FAST(sizeof(float));
      if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseSigma[u_j])
      {
        LOGW(TAG_PPP, "%s pf_phaseSigma malloc failed", __FUNCTION__);
        u_status = 9;
        break;
      }
      pz_filterObs->pz_SatFilterObs[u_i]->u_lsqFlag[u_j] = 0;
    }
  }
  if (u_status)
  {
    return NULL;
  }

  return pz_filterObs;
}

/**
 * @brief free pz_FilterObs
 * @param[in]  pz_FilterObs
 * @return     void
 */
void free_pz_FilterObs(ppp_EpochFilterObs_t** pz_FilterObs)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;

  if (NULL == (*pz_FilterObs))
  {
    return;
  }
  
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == (*pz_FilterObs)->pz_SatFilterObs[u_i])
    {
      continue;
    }
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j]);
      }
      
      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j]);
      }

      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeResidual[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeResidual[u_j]);
      }

      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j]);
      }
      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeVar[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeVar[u_j]);
      }

      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseVar[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseVar[u_j]);
      }

      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeSigma[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_codeSigma[u_j]);
      }

      if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseSigma[u_j])
      {
        OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->pf_phaseSigma[u_j]);
      }
      
    }
    OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]);
  }
  OS_FREE((*pz_FilterObs));
  (*pz_FilterObs) = NULL;
}

/**
 * @brief initilize the PPP algorithm
 * @param[in]  pz_opt represent the algorithm optimization for the PPP-RTK
 * @param[out] void
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t ppp_Algorithm_Init(const ppp_alg_opt_t* pz_opt)
{
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  uint8_t status = 0;
  gz_dopplerDetectPairingBlock.pz_curBlock = NULL;
  gz_dopplerDetectPairingBlock.pz_preBlock = NULL;
  cmn_initTEfilterSet(&gz_gfDetectFilter);
  cmn_initTEfilterSet(&gz_MwDetectFilter);
  cmn_initDopplerDetectCyleSlip(&gz_dopplerDetectPairingBlock);
  //initilize the information of PPP filter
  gz_PPPfilterInfo.w_algStatus = PPP_STATUS_UNKNOW;
  gz_PPPfilterInfo.w_n = 0;
  gz_PPPfilterInfo.u_start = 0;
  gz_PPPfilterInfo.w_nmax = GNSS_MAX_FILTER_STATE_NUM;
  gz_PPPfilterInfo.pd_deltaX = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_PPPfilterInfo.pd_X = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_PPPfilterInfo.pd_Q = (double*)OS_MALLOC_FAST(NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  gz_PPPfilterInfo.pq_paraValid = (BOOL*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(BOOL));

  if (any_Ptrs_Null(4, gz_PPPfilterInfo.pd_deltaX, gz_PPPfilterInfo.pd_X, gz_PPPfilterInfo.pd_Q, gz_PPPfilterInfo.pq_paraValid))
  {
    LOGW(TAG_PPP, "%s X or Q malloc failed", __FUNCTION__);
    OS_FREE(gz_PPPfilterInfo.pd_deltaX);
    OS_FREE(gz_PPPfilterInfo.pd_X);
    OS_FREE(gz_PPPfilterInfo.pd_Q);
    status = 1;
  }
  else
  {
    //initilize the information of EKF status
    tm_initGpsTime(&(gz_EKFstateRepPool.z_gpsTime));
    gz_EKFstateRepPool.d_continueFilterTime = 0.0;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM && !status; ++w_i)
    {
      gz_EKFstateRepPool.pz_satPool[w_i] = (gnss_EKFstateRepresent_t*) OS_MALLOC_FAST(1 * sizeof(gnss_EKFstateRepresent_t));
      if (NULL == gz_EKFstateRepPool.pz_satPool[w_i])
      {
        status = 2;
        break;
      }
      gz_EKFstateRepPool.pz_satPool[w_i]->w_index = INVALID_INDEX;
      initEKFstateRepresent(gz_EKFstateRepPool.pz_satPool[w_i], gz_PPPfilterInfo.w_nmax, gz_PPPfilterInfo.pd_X,
                            gz_PPPfilterInfo.pd_Q);
    }
    gz_PPPfilterInfo.z_kfStatus = KF_INIT;
  }

  //option
  if (NULL != pz_opt)
  {
    gz_PPPfilterInfo.z_opt = *(pz_opt);
  }
  else
  {
    PPP_loadDefaultOption(&(gz_PPPfilterInfo.z_opt));
  }
  
  gz_pppAmbFixInputInfo.pz_EKFstateRepPool = NULL;
  gz_pppAmbFixInputInfo.pz_pppFilterInfo = NULL;
  for (w_i = 0; w_i < GNSS_MAX_AMB_FIXED_TYPE; w_i++)
  {
    gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i] = NULL;
  }
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
      gz_PPPfilterInfo.pq_paraValid[w_i] = FALSE;
  }
  gz_pppAmbFixInputInfo.pz_satSignalPool = NULL;
  gz_pppAmbFixInputInfo.pz_ssrLocBlk = NULL;
  gz_pppAmbFixInputInfo.pz_FilterObs = NULL;
  gz_pppAmbFixInputInfo.u_fixTypeIdx = 0;

  gz_PPPfilterInfo.u_start = status;
  gz_PPPfilterInfo.u_goodSceneStatusFlag = 0;
  gz_PPPfilterInfo.t_openSkyCount = 0;
  gz_PPPfilterInfo.t_closeSkyCount = 0;

  gpz_preTdcpMeasPPP = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  for (w_i = 0; w_i < C_GNSS_MAX; ++w_i)
  {
    for (w_j = 0; w_j < C_GNSS_FREQ_TYPE_MAX; ++w_j)
    {
      gz_PPPfilterInfo.pd_phaseCodeClkDiff[w_i][w_j] = 0.0;
    }
  }

  memset(&gz_PPPfilterInfo.z_iod_count, 0, sizeof(ppp_iod_count_t));

  //initilize gpz_losPredictBlock
  gpz_losPredictBlock = (ppp_LosPredictBlock_t*)OS_MALLOC(sizeof(ppp_LosPredictBlock_t));

  return status;
}

/**
 * @brief deinitilize the PPP algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void ppp_Algorithm_Deinit()
{
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  cmn_deinitTEfilterSet(&gz_gfDetectFilter);
  cmn_deinitTEfilterSet(&gz_MwDetectFilter);
  cmn_deinitDopplerDetectCyleSlip(&gz_dopplerDetectPairingBlock);
  //deinitilize the information of PPP filter
  gz_PPPfilterInfo.w_n = 0;
  gz_PPPfilterInfo.w_nmax = 0;
  gz_PPPfilterInfo.u_start = 0;
  OS_FREE(gz_PPPfilterInfo.pd_deltaX);
  OS_FREE(gz_PPPfilterInfo.pd_X);
  OS_FREE(gz_PPPfilterInfo.pd_Q);
  OS_FREE(gz_PPPfilterInfo.pq_paraValid);

  //deinitilize the information of EKF status
  tm_initGpsTime(&(gz_EKFstateRepPool.z_gpsTime));
  gz_EKFstateRepPool.d_continueFilterTime = 0.0;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    if (NULL != gz_EKFstateRepPool.pz_satPool[w_i])
    {
      OS_FREE(gz_EKFstateRepPool.pz_satPool[w_i]);
    }
  }
  //FILE fp free
  if (gz_PPPfilterInfo.fp_log != NULL)
  {
    fclose(gz_PPPfilterInfo.fp_log);
  }
  if (gz_PPPfilterInfo.fp_nmea != NULL)
  {
    fclose(gz_PPPfilterInfo.fp_nmea);
  }
 
  // free gz_pppAmbFixInputInfo;
  for (w_i = 0; w_i < GNSS_MAX_AMB_FIXED_TYPE; w_i++)
  {
    if (NULL == gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i])
    {
        continue;
    }
    for (w_j = 0; w_j < ALL_GNSS_SYS_SV_NUMBER; w_j++)
    {
      OS_FREE(gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i]->pz_fixedAmbSet[w_j]);
    }
    OS_FREE(gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i]->pd_x_fix);
    OS_FREE(gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i]->pd_q_fix);
    OS_FREE(gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i]);
    gz_pppAmbFixInputInfo.pz_preFixedAmbPool[w_i] = NULL;
  }
  gz_pppAmbFixInputInfo.pz_EKFstateRepPool = NULL;
  gz_pppAmbFixInputInfo.pz_pppFilterInfo = NULL;
  gz_pppAmbFixInputInfo.pz_satSignalPool = NULL;
  gz_pppAmbFixInputInfo.pz_ssrLocBlk = NULL;
  gz_pppAmbFixInputInfo.pz_FilterObs = NULL;

  if (NULL != gpz_preTdcpMeasPPP)
  {
    OS_FREE(gpz_preTdcpMeasPPP);
  }

  if (NULL != gpz_losPredictBlock)
  {
    for (w_i = 0; w_i < DATA_MAX_HIS_COUNT; w_i++)
    {
      for (w_j = 0; w_j < SSR_PREDICT_SAT_NUM_LIMIT; w_j++)
      {
        if (NULL != gpz_losPredictBlock->pz_epochHisSSR[w_i].pz_satHisSSR[w_j])
        {
          OS_FREE(gpz_losPredictBlock->pz_epochHisSSR[w_i].pz_satHisSSR[w_j]);
          gpz_losPredictBlock->pz_epochHisSSR[w_i].pz_satHisSSR[w_j] = NULL;
        }
      }
    }
    OS_FREE(gpz_losPredictBlock);
    gpz_losPredictBlock = NULL;
  }


  return;
}

/**
 * @brief fillup the t_openSkyCount and t_closeSkyCount field in the struct ppp_filterInfo_t
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     void
 */
void PPP_recordSceneInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  gnss_SceneType z_sceneType = OPEN_SKY_SCENE;
  gz_PPPfilterInfo.u_goodSceneStatusFlag = 0;
  gz_PPPfilterInfo.t_openSkyCount <<= 1;
  gz_PPPfilterInfo.t_closeSkyCount <<= 1;
  z_sceneType = gnss_usingObsIdentifyScene(pz_satSigMeasCollect);
  gz_PPPfilterInfo.z_sceneType = z_sceneType;
  if (OPEN_SKY_SCENE == z_sceneType)
  {
    gz_PPPfilterInfo.t_openSkyCount |= 1;
  }
  else if (CLOSE_SKY_SCENE == z_sceneType)
  {
    gz_PPPfilterInfo.t_closeSkyCount |= 1;
  }

  if ((gz_PPPfilterInfo.t_openSkyCount & 0x3) == 0x3)
  {
    gz_PPPfilterInfo.u_goodSceneStatusFlag = 1;
  }
  return;
}

/**
 * @brief extract covariance from filter
 * @param[in] p_Q covariance
 * @param[in] pd_index index
 * @param[in] n number of index
 * @param[in] p_Qout output
 */
static void ppp_RecoverQ(const double* p_Q, const int16_t* pd_index, uint8_t n, double *p_Qout)
{
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      p_Qout[i*n+j] = p_Q[IUTM(pd_index[i], pd_index[j])];
    }
  }
}


/**
 * @brief fill position info
 * @param[in] pz_satSigMeasCollect is the observation information
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in] pz_fixedAmbPool input information for ppp-ar
 * @param[out] pz_pos_solution position struct contains pos/vel and unc of pos/vel
 */
static void ppp_CreatePosSolution(
  const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
  const ppp_filterInfo_t* pz_PPPfilterInfo,
  const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, 
  gnss_PositionFix_t* pz_pos_solution,
  int32_t q_filterStatus)
{
  uint8_t u_i = 0;
  double d_x[3] = {0.0};
  double d_x_coord[3] = {0.0};
  const gnss_PositionFix_t* pz_pos_spp = &pz_satSigMeasCollect->z_positionFix;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };

  // Position solution source from PPP engine
  pz_pos_solution->u_fixSource = FIX_SOURCE_PPP;
  pz_pos_solution->z_gpsTime = pz_pos_spp->z_gpsTime;
  pz_pos_solution->w_size = pz_pos_spp->w_size;


  // PVT
  if (FALSE == pz_PPPfilterInfo->q_filterPosValid || q_filterStatus != 0)
  {
    *pz_pos_solution = *pz_pos_spp;
    pz_pos_solution->u_fixSource = FIX_SOURCE_PPP;
    return;
  }

  double* d_QEcefPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  double* d_QenuPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
  if (any_Ptrs_Null(2, d_QEcefPos, d_QenuPos))
  {
    OS_FREE(d_QEcefPos);
    OS_FREE(d_QenuPos);
    *pz_pos_solution = *pz_pos_spp;
    pz_pos_solution->u_fixSource = FIX_SOURCE_PPP;
    return;
  }

  // Fixed
  if ((NULL != pz_fixedAmbPool) && (GNSS_NONE_AMB_FIXED != pz_fixedAmbPool->u_fixStatus))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      d_x[u_i] = pz_fixedAmbPool->pd_x_fix[u_i];
    }
    int16_t pd_index[3] = {0, 1, 2};
    ppp_RecoverQ(pz_fixedAmbPool->pd_q_fix, pd_index, 3, d_QEcefPos);

    pz_pos_solution->u_fixFlag = GNSS_FIX_FLAG_FIXED;
  }
  // Float
  else
  {
    pz_pos_solution->u_fixFlag = GNSS_FIX_FLAG_FLOATING;
    getPvaParaIndex(&gz_EKFstateRepPool, pw_PVAindex);
    for (u_i = 0; u_i < 3; ++u_i) // pos
    {
      if (-1 != pw_PVAindex[u_i])
      {
        d_x[u_i] = pz_PPPfilterInfo->pd_X[pw_PVAindex[u_i]];
      }
    }
    ppp_RecoverQ(pz_PPPfilterInfo->pd_Q, pw_PVAindex, 3, d_QEcefPos);
  }

  // ENU Q
  gnss_CovEcef2Enu(pz_pos_solution->d_lla, d_QEcefPos, d_QenuPos);

  // Coordinate convert
  gnss_ITRF2WGS84(&pz_satSigMeasCollect->z_tor, d_x, d_x_coord);

  // Position Ecef
  pz_pos_solution->d_xyz[0] = d_x_coord[0];
  pz_pos_solution->d_xyz[1] = d_x_coord[1];
  pz_pos_solution->d_xyz[2] = d_x_coord[2];
  gnss_Ecef2Lla(pz_pos_solution->d_xyz, pz_pos_solution->d_lla);
  pz_pos_solution->f_velXyz[0] = pz_pos_spp->f_velXyz[0];
  pz_pos_solution->f_velXyz[1] = pz_pos_spp->f_velXyz[1];
  pz_pos_solution->f_velXyz[2] = pz_pos_spp->f_velXyz[2];
  pz_pos_solution->f_velEnu[0] = pz_pos_spp->f_velEnu[0];
  pz_pos_solution->f_velEnu[1] = pz_pos_spp->f_velEnu[1];
  pz_pos_solution->f_velEnu[2] = pz_pos_spp->f_velEnu[2];
  // uncertainy pos ecef
  pz_pos_solution->f_posXyzUnc[0] = (float) sqrt(d_QEcefPos[0]);
  pz_pos_solution->f_posXyzUnc[1] = (float) sqrt(d_QEcefPos[4]);
  pz_pos_solution->f_posXyzUnc[2] = (float) sqrt(d_QEcefPos[8]);
  // uncertainy pos lla
  pz_pos_solution->f_posLlaUnc[0] = (float) sqrt(d_QenuPos[0]);
  pz_pos_solution->f_posLlaUnc[1] = (float) sqrt(d_QenuPos[4]);
  pz_pos_solution->f_posLlaUnc[2] = (float) sqrt(d_QenuPos[8]);
  // uncertainy vel enu
  pz_pos_solution->f_velEnuUnc[0] = pz_pos_spp->f_velEnuUnc[0];
  pz_pos_solution->f_velEnuUnc[1] = pz_pos_spp->f_velEnuUnc[1];
  pz_pos_solution->f_velEnuUnc[2] = pz_pos_spp->f_velEnuUnc[2];

  pz_pos_solution->z_dops = pz_satSigMeasCollect->z_positionFix.z_dops;
  tm_cvt_GpstToEpoch(&pz_satSigMeasCollect->z_tor, &pz_pos_solution->z_epoch);
  pz_pos_solution->u_leapsec = tm_LeapSecond_FromEpoch(&pz_pos_solution->z_epoch);
  pz_pos_solution->u_CN040 = pz_pos_spp->u_CN040;
  pz_pos_solution->d_avgCN0 = pz_pos_spp->d_avgCN0;
  pz_pos_solution->d_quasiGeoidHeight = pz_pos_spp->d_quasiGeoidHeight;
  pz_pos_solution->z_rtk.f_age = pz_PPPfilterInfo->f_age;
  memcpy(&pz_pos_solution->z_SvStatus, &pz_pos_spp->z_SvStatus, sizeof(gnss_PositionFix_SvStatus_t));
  pz_pos_solution->z_SvStatus.u_SvInUseCount = pz_PPPfilterInfo->u_ns;

  OS_FREE(d_QEcefPos);
  OS_FREE(d_QenuPos);

  return;
}

/**
 * @brief count the satellite number of havsing LOS correction
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in]  w_status is status of PPP
 * @return     the satellite number of havsing LOS correction
 */
uint8_t PPP_ObsSSRMaskCheck(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,uint32_t* w_status)
{
  uint8_t u_losSatNum = 0;
  uint8_t u_ephSatNum = 0;
  uint8_t u_nbias = 0;
  uint8_t u_allsignal = 0;
  uint8_t u_nstec = 0;
  uint8_t u_allns = 0;
  uint8_t u_ninvalidpr = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t q_satIndex = 0;
  float f_snr_ave = 0.0f;
  float f_stec_qi_ave = 0.0f;
  double d_wave = 1.0;
  const gnss_epochSsrLos_t* pz_epochLos = &(pz_SsrLocBlk->z_epochLosInfo);
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_SignalCorr_t* pz_signal_corr = NULL;

  if (pz_satSigMeasCollect->u_satMeasCount <= 0)
  {
    *w_status |= PPP_STATUS_NO_OBS;
    return 0;
  }

  // check ssr1
  for (u_i = 0; u_i < (pz_epochLos->u_satCount); ++u_i)
  {
    pz_satLos = &(pz_epochLos->z_satLosCorr[u_i]);
    if (tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    if ((pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
    {
      u_ephSatNum++;
    }
    if ((pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)&&
      (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)&&
        (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      u_losSatNum++;
    }
  }

  // check ssr2-3
  for (u_i = 0; u_i < pz_satSigMeasCollect->u_satMeasCount; ++u_i)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_i];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas|| pz_satMeas->z_satPosVelClk.f_elevation*RAD2DEG<10.0)
    {
      continue;
    }
    if (PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_PR) < 1||
      C_GNSS_QZS== pz_satMeas->u_constellation)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    
    u_allns++;
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    if ((pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR) &&
      tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_STECtime) <= PPP_MAX_SSR_AGE
      && pz_satLos->z_stec.f_qi<10.0)
    {
      u_nstec++;
      f_stec_qi_ave += (float)(pz_satLos->z_stec.f_qi * 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(pz_satMeas->u_constellation)));
    }
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_j];
      if (NULL == pz_sigMeas||fabs(pz_sigMeas->d_pseudoRange)<FABS_ZEROS)
      {
        continue;
      }
      if (!pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        u_ninvalidpr++;
      }

      u_allsignal++;
      f_snr_ave += pz_sigMeas->f_cn0;
      pz_signal_corr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
      if (NULL == pz_signal_corr)
      {
        continue;
      }
      if (pz_signal_corr->u_biasMask & GNSS_SSR_SAT_BIAS_CODE_CORR
        && pz_signal_corr->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
      {
        u_nbias++;
      }
    }
  }

  // ssr1 status
  if (u_ephSatNum <= 0)
  {
    *w_status |= PPP_STATUS_NO_NAV;
  }
  if (u_losSatNum <= 0)
  {
    *w_status |= PPP_STATUS_NO_SSR1;
  }

  // ssr2 status
  if (u_allsignal > 0)
  {
    if (u_nbias <= 0)
    {
      *w_status |= PPP_STATUS_NO_SSR2;
    }
    else if (u_nbias < u_allsignal * 0.68)
    {
      *w_status |= PPP_STATUS_LITTLE_SSR2;
    }
  }

  // ssr3 status
  if (u_allns > 0)
  {
    if (u_nstec <= 0)
    {
      *w_status |= PPP_STATUS_NO_SSR3;
    }
    if (u_nstec < u_allns * 0.68)
    {
      *w_status |= PPP_STATUS_few_SSR3;
    }
    if (u_nstec < u_allns * 0.90)
    {
      *w_status |= PPP_STATUS_a_few_SSR3;
    }
  }

  if (u_allsignal > 0)
  {
    f_snr_ave = f_snr_ave / u_allsignal;
    if (f_snr_ave < 35.0)
    {
      *w_status |= PPP_STATUS_SNR_AVE_35;
    }
    if (u_ninvalidpr > u_allsignal * 0.68)
    {
      *w_status |= PPP_STATUS_MUCH_GROSS_SPP;
    }
  }

  if (u_nstec > 0)
  {
    f_stec_qi_ave = f_stec_qi_ave / u_nstec;
    if (f_stec_qi_ave > 0.16)
    {
      *w_status |= PPP_STATUS_STEC_WARNING;
    }
  }
  
  return u_losSatNum;
}
/**
 * @brief initialize pz_pppAmbFixInputInfo
 * @param[in]  pz_satSigMeasCollect
 * @param[in]  pz_SsrLocBlk
 * @param[in]  pz_PPPfilterInfo
 * @param[in]  pz_EKFstateRepPool
 * @param[in]  pz_ssrStatus
 * @param[out]  pz_pppAmbFixInputInfo
 * @return     0 represent success and other failure
 */
static void setAmbFixInputInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo, 
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_EpochFilterObs_t* pz_FilterObs, gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  )
{
 
  pz_pppAmbFixInputInfo->pz_satSignalPool = pz_satSigMeasCollect;
  pz_pppAmbFixInputInfo->pz_ssrLocBlk = pz_SsrLocBlk;
  pz_pppAmbFixInputInfo->pz_pppFilterInfo = pz_PPPfilterInfo;
  pz_pppAmbFixInputInfo->pz_EKFstateRepPool = pz_EKFstateRepPool;

  pz_pppAmbFixInputInfo->pz_FilterObs = pz_FilterObs;

}

static void tracelos(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk) 
{
#if 0

  char id[4] = "",code[3]="";
  int i = 0, j = 0, ns = 0, nbias = 0;
  double dt = 0.0, dt_stec = 0.0, stec = 0.0, var = 0.0, fact = 1.0, trop = 0.0, dt_trop = 0.0, trop_var = 0.0;
  EpochTime_t curtime;
  GpsTime_t gpst;

  static FILE* fp = NULL;
  if (NULL == fp)
  {
    fp = fopen("E:\\los.ssr", "w");
  }
  if (fp == NULL) return;
  gpst = pz_satSigMeasCollect->z_tor;
  tm_GpstTimeAdd(&gpst, 18.0);
  tm_cvt_GpstToEpoch(&gpst, &curtime);

  ns = pz_SsrLocBlk->z_epochLosInfo.u_satCount;
  /* epoch time and ztd */   
  fprintf(fp, "> Epoch %4d %02d %02d %02d %02d %04.1lf %3d     %7.3lf %7.3lf %7.3lf %d\n", (int)curtime.year, (int)curtime.month, (int)curtime.day
    , (int)curtime.hour, (int)curtime.min, curtime.second, ns, pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.d_dryCorr, pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.d_wetCorr
    , pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.f_qi, pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask);

  /* los */ 
  for (i = 0; i < ns; i++)
  {
    stec = 0.0; var = 0.0;
    dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime);
    if (dt > 300.0) dt = 300;

    fact = 1.0;// 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_constellation));
    stec = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_stec.q_corr * 0.001 * fact;
    var = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_stec.f_qi * fact;
    trop = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_std.q_corr * 0.001;
    trop_var = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_std.f_qi;
    
    dt_trop = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_STDtime);
    dt_stec = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_STECtime);
    if (fabs(dt_stec) > 999.9) dt_stec = 999.9;
    if (fabs(dt_trop) > 999.9) dt_trop = 999.9;
    svid_SatString(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_svid, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_constellation, id); 
    fprintf(fp, "%s %5d%14.3lf%14.3lf%14.3lf%14.3lf%14.3lf%14.3lf%14.3lf%14.3lf %14.3lf %7.3lf %6.1lf   %9.3lf %9.3lf %6.1lf   %9.3lf %9.3lf %6.1lf %d\n",
      id, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].q_iode, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satPosClk[0]
      , pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satPosClk[1], pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satPosClk[2]
      , pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satVelClk[0], pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satVelClk[1]
      , pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satVelClk[2], pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satPosClk[3]
      , pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc.d_satVelClk[3], pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_orbClk.q_corr*0.001,
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_orbClk.f_qi*0.001, dt, stec, var, dt_stec, trop,trop_var, dt_trop, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_windUpMask);
  }

  /* OSB bias */   
  //if (((int)floor((ep[4] * 600 + ep[5]) * 10 + 0.5)) % 100 == 0)
  if (1)
  {         
    /* CODE bias */ 
    fprintf(fp, "> code bias %4d %02d %02d %02d %02d %04.1lf %6.1lf %3d\n", (int)curtime.year, (int)curtime.month, (int)curtime.day
      , (int)curtime.hour, (int)curtime.min, curtime.second, 1.0, ns);   
    for (i = 0; i < ns; i++) 
    {
      dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime);
      nbias = 0; 
      for (j = 0; j < M_ARRAY_SIZE(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr); j++)
      {
        if (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask & GNSS_SSR_SAT_BIAS_CODE_CORR)
        {
          nbias++;
        }
      }
      if (nbias <= 0) continue;
      svid_SatString(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_svid, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_constellation, id);
      fprintf(fp, "%s %2d", id, nbias);  
      for (j = 0; j < M_ARRAY_SIZE(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr); j++)
      { 
        if (!(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask & GNSS_SSR_SAT_BIAS_CODE_CORR))
        {
          continue;
        }
        gnss_cvt_Sig2CodeStr(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_signalType, code);
        fprintf(fp, " %s %7.3f", code, -pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_codeBias.q_corr * 0.001); 
      }
      fprintf(fp, " %7.2lf\n", dt);
    }
    /* phase bias */ 
    fprintf(fp, "> phase bias %4d %02d %02d %02d %02d %04.1lf %6.1lf %3d\n", (int)curtime.year, (int)curtime.month, (int)curtime.day
      , (int)curtime.hour, (int)curtime.min, curtime.second, 1.0, ns); 
    for (i = 0; i < ns; i++)
    {
      dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime);
      nbias = 0;
      for (j = 0; j < M_ARRAY_SIZE(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr); j++)
      {
        if (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
        {
          nbias++;
        }
      }
      if (nbias <= 0) continue;
      svid_SatString(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_svid, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].u_constellation, id);
      fprintf(fp, "%s %8.3lf %8.3lf %8.3lf %5d", id, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].f_yaw, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].f_yawRate, 
        pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].d_windUp, nbias);
      for (j = 0; j < M_ARRAY_SIZE(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr); j++)
      {
        if (!(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
        {
          continue;
        }
        gnss_cvt_Sig2CodeStr(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_signalType, code);
        fprintf(fp, " %s %7.3f", code, -pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_phaseBias.q_corr * 0.001);
      }
      fprintf(fp, " %7.2lf\n", dt);
    }
   }

#endif // 0

}
static void traceobs(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
#if 0

  char id[4] = "";
  int i = 0, j = 0, ns = 0;
  EpochTime_t curtime;
  GpsTime_t gpst;
  gnss_SignalMeas_t* pz_signalMeas = NULL;

  static FILE* fp = NULL;
  if (NULL == fp)
  {
    fp = fopen("E:\\log.23O", "w");
  }
  if (fp == NULL) return;
  gpst = pz_satSigMeasCollect->z_tor;
  tm_GpstTimeAdd(&gpst, 18.0);
  tm_cvt_GpstToEpoch(&gpst, &curtime);

  ns = pz_satSigMeasCollect->u_satMeasCount;

  /* epoch time */
  fprintf(fp, "> %4d %02d %02d %02d %02d %10.7lf 0%3d\n", (int)curtime.year, (int)curtime.month, (int)curtime.day
    , (int)curtime.hour, (int)curtime.min, curtime.second, ns);

  for (i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++)
  {
    if (NULL == pz_satSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }
    svid_SatString(pz_satSigMeasCollect->pz_satMeas[i]->u_svid, pz_satSigMeasCollect->pz_satMeas[i]->u_constellation, id);
    fprintf(fp, "%3s", id);
    for (j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
    {
      if (NULL == pz_satSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j])
      {
        fprintf(fp, "%14.3lf  %14.3lf%d  %14.3lf  %14.3lf", 0.0, 0.0, 3, 0.0, 0.0);
        continue;
      }
      pz_signalMeas = pz_satSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j];
      fprintf(fp, "%14.3lf  %14.3lf%d  %14.3lf  %14.3lf", pz_signalMeas->d_pseudoRange, pz_signalMeas->d_carrierPhase, pz_signalMeas->u_LLI, -pz_signalMeas->d_doppler/wavelength(pz_signalMeas->u_signal), pz_signalMeas->f_cn0);
    }
    fprintf(fp, "\n");
  }

#endif // 0

}

/**
 * @brief print ssr mask log
 * @param[in] pz_SsrLocBlk SSR block
 */
void PPP_logSSRMask(const gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  char satid[4] = {0};
  double dt = 0.0;

  /* orbclk, atmo, windup */
  LOGI(TAG_PPP, "ns=%2d;ztd-%d;  ssr mask: orbclk, atmo, windup, iode, dt:\n",
    pz_SsrLocBlk->z_epochLosInfo.u_satCount, pz_SsrLocBlk->z_epochLosInfo.u_ZTDmask);
  for (int u_i = 0; u_i < (pz_SsrLocBlk->z_epochLosInfo.u_satCount); ++u_i)
  {
    dt = 0.0;
    const gnss_satSsrLos_t* pz_satLos = &(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i]);
    if (pz_satLos->z_orbClkTime.q_towMsec != 0 && pz_satLos->z_orbClkTime.w_week != 0)
    {
      dt = tm_GpsTimeDiff(&pz_SsrLocBlk->z_epochLosInfo.z_tor, &pz_satLos->z_orbClkTime);
    }
    svid_SatString(pz_satLos->u_svid, pz_satLos->u_constellation, satid);
    LOGI(TAG_PPP, "%s, %d, %d, %d, %3d, %.1f\n", satid, pz_satLos->u_orbClkMask, pz_satLos->u_atmoMask,
         pz_satLos->u_windUpMask, pz_satLos->q_iode, dt);
  }
}

/**
 * @brief contrast sat pos between SSR with ALGO
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 */
void PPP_contrastSatPosBetweenSSRandAlg(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }

  char* buff = OS_MALLOC_FAST(1024);
  if (NULL == buff)
  {
    return;
  }
  char* p = buff;
  char satid[4] = {0};

  for (int i = 0; i < pz_satSigMeasCollect->u_satMeasCount; ++i)
  {
    uint8_t sidx = pz_satSigMeasCollect->u_satMeasIdxTable[i];
    const gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[sidx];
    const gnss_satSsrLos_t* pz_ssr = getSSR_constLos(pz_meas->u_constellation, pz_meas->u_svid, pz_SsrLocBlk);
    if ((pz_ssr != NULL) &&
        (1 == (pz_ssr->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)) &&
        (pz_meas->z_satPosVelClk.q_iode != -1))
    {
      // ply-ssr
      double delta_pos[4] = {0.0};
      double delta_vel[4] = {0.0};
      for (uint8_t j = 0; j < 3; ++j)
      {
        delta_pos[j] = pz_meas->z_satPosVelClk.d_satPosClk[j] - pz_ssr->z_satPosVelClkBrdc.d_satPosClk[j];
      }
      delta_pos[3] = pz_meas->z_satPosVelClk.d_satPosClk[3] * CLIGHT - pz_ssr->z_satPosVelClkBrdc.d_satPosClk[3];

      for (uint8_t j = 0; j < 4; ++j)
      {
        delta_vel[j] = pz_meas->z_satPosVelClk.d_satVelClk[j] - pz_ssr->z_satPosVelClkBrdc.d_satVelClk[j];
      }

      // log
      memset(buff, 0, 1024); // reset buff
      p = buff; // reset buff
      satidx_SatString(sidx, satid);
      double all_d = gnss_Norm(delta_pos, 4);

      p += snprintf(p, 300, "%7.1f, %s, %.1f, %4d, %4d",
                    pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV,
                    satid,
                    pz_meas->z_satPosVelClk.f_dt,
                    pz_meas->z_satPosVelClk.q_iode,
                    pz_ssr->q_iode);
      p += snprintf(p, 300, ",%10.4f, %10.4f, %10.4f, %10.4f, %9.4f, %9.3f, %9.3f, %9.3f, %6.3f\n", all_d, delta_pos[0],
                    delta_pos[1], delta_pos[2], delta_pos[3],
                    delta_vel[0], delta_vel[1], delta_vel[2], delta_vel[3]);

      DATA_LOG(DLOG_SAT_POS, "%s", buff);
    }
  }
  OS_FREE(buff);
}
/**
 * @brief get value of SSR
 * @param[in]   pz_SsrLocBlk
 * @param[out]   f_orb_mindt
 * @param[out]   f_orb_meanqi
 * @param[out]   f_stec_meanqi
 */
static void getMeanValueSSR(const gnss_ssrLosBlock_t* pz_SsrLocBlk, float* f_orb_mindt, float* f_orb_meanqi, float* f_stec_meanqi)
{
  uint8_t u_i = 0;
  uint8_t u_orb[C_GNSS_MAX] = { 0 };
  uint8_t u_stec[C_GNSS_MAX] = { 0 };
  gnss_ConstellationType u_constellation = 0;
  double dt = 0.0;
  float* f_orb_qi[C_GNSS_MAX] = { NULL };
  float* f_stec_qi[C_GNSS_MAX] = { NULL };

  for (u_i = 0; u_i < C_GNSS_MAX; u_i++)
  {
    f_orb_mindt[u_i] = 3600.0;
    f_orb_qi[u_i] = (float*)OS_MALLOC_FAST(sizeof(float) * GNSS_SSR_SAT_NUM_LIMIT);
    f_stec_qi[u_i] = (float*)OS_MALLOC_FAST(sizeof(float) * GNSS_SSR_SAT_NUM_LIMIT);
  }

  for (u_i = 0; u_i < GNSS_SSR_SAT_NUM_LIMIT; u_i++)
  {
    if (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid <= 0)
    {
      continue;
    }
    if (0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
      || 0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || 0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    u_constellation = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation;
    if (NULL == f_orb_qi[u_constellation] || NULL == f_stec_qi[u_constellation])
    {
      continue;
    }
    f_orb_qi[u_constellation][u_orb[u_constellation]++] = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.f_qi * 0.001f;
    dt = tm_GpsTimeDiff(&pz_SsrLocBlk->z_epochLosInfo.z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClkTime);
    if (dt < f_orb_mindt[u_constellation])
    {
      f_orb_mindt[u_constellation] = (float)dt;
    }
    if (!(GNSS_SSR_ATMO_STEC_CORR & pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_atmoMask))
    {
      continue;
    }
    f_stec_qi[u_constellation][u_stec[u_constellation]++] = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_stec.f_qi;
  }

  for (u_i = 0; u_i < C_GNSS_MAX; u_i++)
  {
    if (u_orb[u_i] >= 3)
    {
      gnss_ascSortMedianFloat(f_orb_qi[u_i], u_orb[u_i], &f_orb_meanqi[u_i]);
    }
    if (u_stec[u_i] >= 3)
    {
      gnss_ascSortMedianFloat(f_stec_qi[u_i], u_stec[u_i], &f_stec_meanqi[u_i]);
    }

    OS_FREE(f_orb_qi[u_i]); 
    OS_FREE(f_stec_qi[u_i]);
  }
}
/**
 * @brief pre-processing of SSR
 * @param ssr
 */
static void QICheckSSR(gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  uint8_t u_i = 0;
  gnss_ConstellationType u_constellation = 0;
  char char_sat[4] = "";
  float f_qi = 0.0;
  double dt = 0.0;
  float f_orb_mindt[C_GNSS_MAX] = { 0.0 };
  float f_orb_meanqi[C_GNSS_MAX] = { 0.0 };
  float f_stec_meanqi[C_GNSS_MAX] = { 0.0 };

  getMeanValueSSR(pz_SsrLocBlk, f_orb_mindt, f_orb_meanqi, f_stec_meanqi);

  for (u_i = 0; u_i < GNSS_SSR_SAT_NUM_LIMIT; u_i++)
  {
    if (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid <= 0)
    {
      continue;
    }
    if (0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
      || 0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || 0 == (pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid))
    {
      continue;
    }
    u_constellation = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation;
    svid_SatString(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, char_sat);
    /*dt = tm_GpsTimeDiff(&pz_SsrLocBlk->z_epochLosInfo.z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClkTime);
    if (fabs(dt - f_orb_mindt[u_constellation]) > (1.0e-4) && fabs(f_orb_mindt[u_constellation]) > (1.0e-4))
    {
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.f_qi = 999999.9f;
      LOGI(TAG_PPP, "Wrong epoch los time,sat=%s  dt=%.1lf  mindt=%.1f\n", char_sat, dt, f_orb_mindt[u_constellation]);
    }*/
    f_qi = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.f_qi * 0.001f;
    if (f_qi >= f_orb_meanqi[u_constellation] * 3.0 && f_qi > 0.08 && fabs(f_orb_meanqi[u_constellation]) > FABS_ZEROS)
    {
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.f_qi *= 2.0;
      LOGI(TAG_PPP, "Abmormal qi of los,sat=%s  qi=%.3f  meanqi=%.3f\n", char_sat, f_qi, f_orb_meanqi[u_constellation]);
    }
    //if (!(GNSS_SSR_ATMO_STEC_CORR & pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_atmoMask))
    //{
    //  continue;
    //}
    //f_qi = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_stec.f_qi;
    //if (f_qi * 10.0 < f_stec_meanqi[u_constellation])
    //{
    //  pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_stec.f_qi = f_stec_meanqi[u_constellation];
    //  LOGI(TAG_PPP, "Abmormal qi of stec,sat=%s  qi=%.3f  meanqi=%.3f\n", char_sat, f_qi, f_stec_meanqi[u_constellation]);
    //}
  }
}



/**
 * @brief Test sat position form brdc
 * @param gpst
 */
static void ppp_AlgorithmTestSatpos(const GpsTime_t* gpst)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    char id[4] = {0};
    gnss_SatPosVelClk_t z_satPosVelClk = {0};
    uint8_t svid = 0;
    uint8_t cons = 0;
    svid_SatString(svid,cons,id);
    svid = gnss_cvt_SvIndex2Svid(i, &cons);
    if (sd_api_SatPositionFromBrdc(gpst, cons, svid, -1, TRUE, &z_satPosVelClk))
    {
      DATA_LOG(DLOG_SAT_POS, "%d, %s, %d, %15.3f\n", gpst->q_towMsec * TIME_MSEC_INV, id, z_satPosVelClk.q_iode,
               z_satPosVelClk.d_satPosClk[3]);
    }
  }
}

/**
 * @brief Convert orbit delta dxyz to los by ephemeris satellite position
 * @param[in] d_roverPos receiver coordinate ecef [xyz]
 * @param[in] d_satPos satellite ephemeris position ecef [xyz]
 * @param[in] q_delta_pos  precision oribit, relative to EPH dx/dy/dz
 * @return double LOS
 */
static double ppp_calOrbitLosFromDeltaXYZ(const double* d_roverPos, const double* d_satPos, const int32_t* q_delta_pos)
{
  double d_ephSatPosRot[3] = {0.0};
  double d_preSatPosRot[3] = {0.0};
  double d_preSatPos[3] = {0.0};
  double d_ephRange = 0.0;
  double d_preRange = 0.0;
  double d_eEph[3] = {0.0};
  double d_ePre[3] = {0.0};
  double los = 0.0;

  d_preSatPos[0] = d_satPos[0] + (double)q_delta_pos[0] * 0.001;
  d_preSatPos[1] = d_satPos[1] + (double)q_delta_pos[1] * 0.001;
  d_preSatPos[2] = d_satPos[2] + (double)q_delta_pos[2] * 0.001;

  gnss_satRot(d_roverPos, d_satPos, d_ephSatPosRot);
  gnss_satRot(d_roverPos, d_preSatPos, d_preSatPosRot);


  d_eEph[0] = d_roverPos[0] - d_ephSatPosRot[0];
  d_eEph[1] = d_roverPos[1] - d_ephSatPosRot[1];
  d_eEph[2] = d_roverPos[2] - d_ephSatPosRot[2];
  d_ePre[0] = d_roverPos[0] - d_preSatPosRot[0];
  d_ePre[1] = d_roverPos[1] - d_preSatPosRot[1];
  d_ePre[2] = d_roverPos[2] - d_preSatPosRot[2];

  d_ephRange = gnss_Norm(d_eEph, 3);
  d_preRange = gnss_Norm(d_ePre, 3);

  los = d_preRange - d_ephRange;
  return los;
}

/**
 * @brief Precision orbit correction and clock bias convert to LOS + ephemeris
 * @param[in] pz_satSigMeasCollect
 * @param[in,out] pz_SsrLocBlk
 */
static void ppp_ssrOrbitClock2Los(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                                         gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  double d_tc = 0.0;
  double d_af1 = 0.0;
  int32_t q_drift = 0;
  int32_t q_rela_corc = 0;
  gnss_Ephemeris_t* pz_eph = (gnss_Ephemeris_t*)OS_MALLOC_FAST(sizeof(gnss_Ephemeris_t));

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[i];
    if (NULL == pz_meas) // pz_meas is NULL
    {
      continue;
    }
    gnss_satSsrLos_t* pz_los = getSSR_Los(pz_meas->u_constellation, pz_meas->u_svid, pz_SsrLocBlk);
    if(NULL == pz_los) // no los
    {
      continue;
    }
    if (!M_IS_SET_MASK(pz_los->u_orbClkMask, GNSS_SSR_SAT_ORB_CLK_PRE_ORBIT | GNSS_SSR_SAT_ORB_CLK_PRE_CLOCK))
    {
      continue;
    }

    /* orbit los */
    {
      if(pz_los->q_iode == pz_meas->z_satPosVelClk.q_iode)
      {
        memcpy(&pz_los->z_satPosVelClkBrdc, &pz_meas->z_satPosVelClk, sizeof(gnss_SatPosVelClk_t));
      }
      else
      {
        if (!sd_api_SatPositionFromBrdc(&pz_meas->z_tot, pz_meas->u_constellation, pz_meas->u_svid, pz_los->q_iode, TRUE, &pz_los->z_satPosVelClkBrdc))
        {
          continue;
        }
      }
      double los = ppp_calOrbitLosFromDeltaXYZ(pz_satSigMeasCollect->z_positionFix.d_xyz, pz_los->z_satPosVelClkBrdc.d_satPosClk, pz_los->z_orbclkPrec.q_delta_pos);
      pz_los->z_orbClk.q_corr = (int32_t)round(los * 1000.0);
      pz_los->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_EPH;
      pz_los->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR;
      pz_los->z_orbClk.f_qi = pz_los->z_orbclkPrec.f_qi;
      pz_los->z_orbClk.u_preItg = pz_los->z_orbclkPrec.u_preItg;
      pz_los->z_orbClk.u_postItg = pz_los->z_orbclkPrec.u_postItg;
    }
    /* clock los */
    {
      /* relativity correction */
      // q_rela_corc = (int32_t)round(-2.0 * gnss_Dot(pz_los->z_satPosVelClkBrdc.d_satPosClk, pz_los->z_satPosVelClkBrdc.d_satVelClk, 3) / CLIGHT * 1000.0);

      /* clock drift */
#if defined(APP_HEXAGON_SSR)
      if(!sd_api_GnssEphemeris_Get(NULL, pz_meas->u_constellation, pz_meas->u_svid, pz_los->q_iode, pz_eph))
      {
        continue;
      }
      
      if(pz_meas->z_tot.w_week !=0)
      {
        d_tc = tm_GpsTimeDiff(&pz_meas->z_tot, &pz_los->z_orbClkTime);
      }
      else{
        d_tc = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_los->z_orbClkTime);
      }
      d_af1 = 0.0;
      switch (pz_meas->u_constellation)
      {
      case C_GNSS_GPS:
      case C_GNSS_QZS:
        d_af1 = pz_eph->eph.z_gpsEph.af1;
        break;
      case C_GNSS_GAL:
        d_af1 = pz_eph->eph.z_galEph.af1;
        break;
      case C_GNSS_BDS2:
      case C_GNSS_BDS3:
        d_af1 = pz_eph->eph.z_bdsEph.af1;
        break;
      default:
        d_af1 = 0.0;
        break;
      }
      if(d_af1 == 0.0)
      {
        continue;  
      }
      q_drift = (int32_t)round(d_af1 * d_tc * CLIGHT * 1000.0);
#endif // APP_HEXAGON_SSR

      pz_los->z_orbClk.q_corr -= (pz_los->z_orbclkPrec.q_clock_bias + q_rela_corc + q_drift -
            (int32_t)round(pz_los->z_satPosVelClkBrdc.d_satPosClk[3] * 1000.0));
      pz_los->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR;
      }
    }
  OS_FREE(pz_eph);
}

/**
 * @brief replace satellite position in SSR with satellite position from rover ephemeris
 * @param[in] pz_satSigMeasCollect
 * @param[in,out] pz_SsrLocBlk
 */
static void ppp_replaceSSRSatPosition(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                                         gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  char c_sat[4] = "";
  double dft = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_tor);
  gnss_SatPosVelClk_t* pz_satPosVelClk = OS_MALLOC_FAST(sizeof(gnss_SatPosVelClk_t));
  if (NULL == pz_satPosVelClk)
  {
    LOGW(TAG_PPP, "Replace SSR sat pos error, because malloc failed\n");
    for (int j = 0; j < pz_SsrLocBlk->z_epochLosInfo.u_satCount; j++)
    {
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[j].u_orbClkMask = 0;
    }
    return;
  }

  /* replace */
  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[i];
    if (NULL == pz_meas)
    {
      continue; // pz_meas is NULL
    }
    // find ssr
    gnss_satSsrLos_t* pz_los = getSSR_Los(pz_meas->u_constellation, pz_meas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_los)
    {
      continue;
    }
    if (fabs(dft) < FABS_ZEROS && (pz_los->u_orbClkMask& GNSS_SSR_SAT_ORB_CLK_EPH)
      ||(!pz_los->u_orbClkMask|| pz_los->q_iode < 0))
    {
      continue; // SSR-MEAS time > 0
    }
    memset(pz_satPosVelClk, 0, sizeof(gnss_SatPosVelClk_t));
    memset(&pz_los->z_satPosVelClkBrdc, 0, sizeof(gnss_SatPosVelClk_t));
    if (sd_api_SatPositionFromBrdc(&pz_meas->z_tot, pz_meas->u_constellation, pz_meas->u_svid, pz_los->q_iode, TRUE,
      pz_satPosVelClk))
    {
      pz_los->u_orbClkMask |= GNSS_SSR_SAT_ORB_CLK_EPH;
      memcpy(&pz_los->z_satPosVelClkBrdc, pz_satPosVelClk, sizeof(gnss_SatPosVelClk_t));
    }
    else
    {
      svid_SatString(pz_meas->u_svid, pz_meas->u_constellation, c_sat);
      LOGI(TAG_PPP, "Replace SSR sat pos fail %s iode=%d\n", c_sat, pz_los->q_iode);
      pz_los->q_iode = -1;
      pz_los->u_orbClkMask = 0;
    }
  }
  OS_FREE(pz_satPosVelClk);
}

/*****************************************************************************
* Name        : gravitationalDelayCorrection
* Description : Obtains the gravitational delay correction for the effect of
*               general relativity (red shift) to the GPS signal
* Parameters  :
* Name                           |Da|Unit|Description
* double  *rr      I  m    Position of the receiver
* double  *rs      I  m    Position of the satellite
* Returned value (double)         O  m    Gravitational delay correction
*****************************************************************************/
static double gravitationalDelayCorrection(const int sys, const double* rr,
  const double* rs)
{
  double receiverModule;
  double satelliteModule;
  double distance;
  double MU = MU_GPS;
  double delay = 0.0;

  receiverModule = gnss_Norm(rr, 3);
  satelliteModule = gnss_Norm(rs, 3);
  distance = sqrt((rs[0] - rr[0]) * (rs[0] - rr[0]) +
    (rs[1] - rr[1]) * (rs[1] - rr[1]) +
    (rs[2] - rr[2]) * (rs[2] - rr[2]));

  switch (sys) {
  case C_GNSS_GPS:
    MU = MU_GPS;
    break;
  case C_GNSS_GLO:
    MU = MU_GLO;
    break;
  case C_GNSS_GAL:
    MU = MU_GAL;
    break;
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
    MU = MU_BDS;
    break;
  default:
    MU = MU_GPS;
    break;
  }
  delay = 2.0 * MU / (CLIGHT * CLIGHT) * log((satelliteModule + receiverModule + distance) / (satelliteModule + receiverModule - distance));
  return delay;
}
/* phase windup computation */
static void ppp_gravitationDelay(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_ssrLocBlk)
{
  const gnss_satSsrLos_t* pz_satLos = NULL;
  for (uint16_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[i];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (gnss_Norm(pz_satMeas->z_satPosVelClk.d_satPosClk, 3) <= 0.0)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_ssrLocBlk);
    if (NULL == pz_satLos || (pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR))
    {
      continue;
    }
    pz_satMeas->d_Gravitation = gravitationalDelayCorrection(pz_satMeas->u_constellation, pz_satSigMeasCollect->z_positionFix.d_xyz,
      pz_satMeas->z_satPosVelClk.d_satPosClk);
  }
}

/* phase windup computation */
static void ppp_calWindup(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_ssrLosBlock_t* pz_ssrLocBlk)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  char c_sat[4] = "";
  uint32_t sat_idx = 0;
  double rs[6] = { 0.0 };
  double d_phw = 0.0;
  gnss_satSsrLos_t* pz_satLos = NULL;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;

  for (u_j = 0; u_j < pz_ssrLocBlk->z_epochLosInfo.u_satCount; u_j++)
  {
    if (pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_j].u_windUpMask& GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID)
    {
      return; // windup is corrected
    }
  }

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_i];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    pz_satLos = NULL;
    /* Find LOS */
    for (u_j = 0; u_j < pz_ssrLocBlk->z_epochLosInfo.u_satCount; u_j++)
    {
      if (pz_satMeas->u_svid == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_j].u_svid
          && pz_satMeas->u_constellation == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_j].u_constellation)
      {
        pz_satLos = &pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_j];
        break;
      }
    }
    if(NULL == pz_satLos ||!(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
    {
      continue;
    }
    if ((pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID))
    {
      continue;
    }
    sat_idx = gnss_cvt_Svid2SvIndex(pz_satMeas->u_svid, pz_satMeas->u_constellation);

    for (u_k = 0; u_k < 3; ++u_k)
    {
      rs[u_k] = pz_satLos->z_satPosVelClkBrdc.d_satPosClk[u_k];
      rs[u_k + 3] = pz_satLos->z_satPosVelClkBrdc.d_satVelClk[u_k];
    }
#if 1
    if (!model_phw(&pz_satSigMeasCollect->z_tor, sat_idx + 1, "", 1, rs, pz_satSigMeasCollect->z_positionFix.d_xyz, &d_phw))
    {
      continue;
    }
#else
    wup_getWindUp(&pz_satSigMeasCollect->z_tor, pz_satMeas->u_svid, pz_satMeas->u_constellation, NULL,
      pz_satLos->z_satPosVelClkBrdc.d_satPosClk, pz_satLos->z_satPosVelClkBrdc.d_satVelClk,
      pz_satSigMeasCollect->z_positionFix.d_xyz, &d_phw);
#endif // 1
    satidx_SatString(sat_idx, c_sat);
    LOGD(TAG_PPP, "Phw sat=%s phw=%6.3lf %6.3lf diff=%6.3lf\n", c_sat,pz_satLos->d_windUp, d_phw, pz_satLos->d_windUp - d_phw);

    pz_satLos->d_windUp = d_phw;
    pz_satLos->u_windUpMask |= GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID;
  }
}

static void resetfixedSignalAmbPool(gnss_fixedSignalAmbPool_t* fixedSignalAmbPool)
{
  uint8_t u_i = 0;

  memset(&fixedSignalAmbPool->z_time, 0, sizeof(GpsTime_t));
  fixedSignalAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
  fixedSignalAmbPool->u_ambfix_mode = 0;;
  memset(fixedSignalAmbPool->pw_refSat, 0, sizeof(fixedSignalAmbPool->pw_refSat));
  fixedSignalAmbPool->w_continuefixedEpoch = 0;
  memset(fixedSignalAmbPool->u_fixedns, 0, sizeof(fixedSignalAmbPool->u_fixedns));
  fixedSignalAmbPool->u_nb = 0;
  fixedSignalAmbPool->u_gap = 0;
  fixedSignalAmbPool->f_ratio = 0.0;
  fixedSignalAmbPool->f_adop = 0.0;
  fixedSignalAmbPool->f_pdop = 0.0;
  fixedSignalAmbPool->f_cn0Thres = 0.0;
  fixedSignalAmbPool->f_eleThres = 0.0;
  if (NULL != fixedSignalAmbPool->pd_x_fix)
  {
    memset(fixedSignalAmbPool->pd_x_fix, 0, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  }
  if (NULL != fixedSignalAmbPool->pd_q_fix)
  {
    memset(fixedSignalAmbPool->pd_q_fix, 0, NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  }

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == fixedSignalAmbPool->pz_fixedAmbSet[u_i])
    {
      continue;
    }
    memset(fixedSignalAmbPool->pz_fixedAmbSet[u_i], 0, sizeof(gnss_fixedSignalAmb_t));
  }
}

/**
 * @brief reset filter of PPP deci
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[out]     pz_pppAmbFixInputInfo
 * @param[out]     pz_EKFstateRepPool
 * @param[out]     pz_PPPfilterInfo
 * @return         none
 */
static void PPP_breakFilterOption(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_filterInfo_t* pz_PPPfilterInfo)
{
#ifdef RESET_FILTER
  uint8_t u_i = 0;
  uint32_t w_mod = 0;
  uint32_t w_break = 10 * 60; // reset every ten minutes
  static uint32_t w_last_mod = 86400;

  w_mod = ((int)(pz_satSigMeasCollect->z_tor.q_towMsec / 1000.0)) % (w_break);
  if (w_mod < w_last_mod)
  {
    pz_PPPfilterInfo->z_kfStatus = KF_RESET;

    for (u_i = 0; u_i < GNSS_MAX_AMB_FIXED_TYPE; u_i++)
    {
      if (NULL == pz_pppAmbFixInputInfo->pz_preFixedAmbPool[u_i])
      {
        continue;
      }
      resetfixedSignalAmbPool(pz_pppAmbFixInputInfo->pz_preFixedAmbPool[u_i]);
    }

    memset(&pz_EKFstateRepPool->z_gpsTime, 0, sizeof(GpsTime_t));
    pz_EKFstateRepPool->d_continueFilterTime = 0.0;
    for (u_i = 0; u_i < GNSS_MAX_FILTER_STATE_NUM; u_i++)
    {
      if (NULL == pz_EKFstateRepPool->pz_satPool[u_i])
      {
        continue;
      }
      memset(pz_EKFstateRepPool->pz_satPool[u_i], 0, sizeof(gnss_EKFstateRepresent_t));
      pz_EKFstateRepPool->pz_satPool[u_i]->w_index = -1;
    }

    LOGI(TAG_PPP, "Reset PPP filter by option, %dS\n", w_break);
  }
  w_last_mod = w_mod;

#endif // RESET_FILTER
}

void ppp_AlgoSoltag(ppp_filterInfo_t* pz_filter_info, const gnss_PositionFix_t* pz_PositionFix)
{
  switch (pz_PositionFix->u_fixFlag)
  {
  case GNSS_FIX_FLAG_SPS:
    pz_filter_info->u_solTag = GNSS_FILTER_POS_SPP;
    break;
  case GNSS_FIX_FLAG_FLOATING:
    pz_filter_info->u_solTag = GNSS_FILTER_POS_FLOAT;
    break;
  case GNSS_FIX_FLAG_FIXED:
    pz_filter_info->u_solTag = GNSS_FILTER_POS_FIX;
    break;
  }
}

/**
 * @brief checking of ssr
 * @param[in] pz_satSigMeasCollect
 * @param[in,out] pz_SsrLocBlk
 */
static void ppp_ssrProcess(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  /********* Orbit clock *********/
  /* precisiong orbit clokc to LOS */
  ppp_ssrOrbitClock2Los(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* Los forcast of SSR*/
  ppp_LosPredictSSR(pz_satSigMeasCollect, pz_SsrLocBlk, &pz_satSigMeasCollect->z_tor, gpz_losPredictBlock);

  /* satellite position from BRDC replace SSR */
  ppp_replaceSSRSatPosition(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* contrast sat pos between SSR with ALGO */
  PPP_contrastSatPosBetweenSSRandAlg(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* Abnormal qi of ssr detection */
  if (fabs(tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_tor)) <= FABS_ZEROS)
  {
    QICheckSSR(pz_SsrLocBlk); // pz_SsrLocBlk may be modified
  }

  /* mask of obs and ssr computation */
  PPP_ObsSSRMaskCheck(pz_satSigMeasCollect, pz_SsrLocBlk, &gz_PPPfilterInfo.w_algStatus);

}

/**
 * @brief Some PPP model delay computation
 * @param[in] pz_satSigMeasCollect
 * @param[in,out] pz_SsrLocBlk
 */
static void ppp_modelDelay(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  /********* Some PPP model delay computation *********/
  /* earth tide */
  tide_Displacement(&pz_satSigMeasCollect->z_tor, pz_satSigMeasCollect->z_positionFix.d_xyz, pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask
    , NULL, NULL, pz_satSigMeasCollect->f_earthTide);

  /* windup */
  ppp_calWindup(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* gravitation delay */
  ppp_gravitationDelay(pz_satSigMeasCollect, pz_SsrLocBlk);
}
/**
 * @brief the inteface of prococess the PPP-RTK algorithm
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @return     status mask of ppp algorithm
 */
uint32_t ppp_Algorithm_Prcocess(gnss_ssrLosBlock_t* pz_SsrLocBlk, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                               gnss_PositionFix_t* pz_PositionFix)
{
  int32_t q_status = 0;
  ppp_EpochFilterObs_t* pz_filterObs = NULL;
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool = NULL;
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect_prev = ppp_GetPreviousSatSigMeasCollect();
  gnss_TdcpMeasBlock_t* pz_curDcpMeas = NULL;
  gnss_TdcpMeasPair_t z_tdcpMeasPointerPair = { NULL,NULL };

  gz_PPPfilterInfo.w_algStatus = PPP_STATUS_UNKNOW;
  if (any_Ptrs_Null(4, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PositionFix, pz_SatSigMeasCollect_prev))
  {
    gz_PPPfilterInfo.w_algStatus |= PPP_STATUS_MEMORY_ERROR;
    return gz_PPPfilterInfo.w_algStatus;
  }
  PPP_recordSceneInfo(pz_satSigMeasCollect);

  /* process ssr */
  ppp_ssrProcess(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* Some PPP model delay computation */
  ppp_modelDelay(pz_satSigMeasCollect, pz_SsrLocBlk);

  /* log modifed ssr */
  PPP_logSSRMask(pz_SsrLocBlk);

  if (gz_PPPfilterInfo.w_algStatus & PPP_STATUS_NO_SSR1)
  {
    return gz_PPPfilterInfo.w_algStatus;
  }
  
  pz_filterObs = malloc_pz_ssrStatus(pz_satSigMeasCollect);
  malloc_fixedSignalAmbPool(&pz_fixedAmbPool, pz_satSigMeasCollect);

  /* Debug time */
  EpochTime_t z_epochT;
  double d_tow = pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV;
  tm_cvt_GpstToEpoch(&pz_satSigMeasCollect->z_tor, &z_epochT);
  LOGI(TAG_PPP, "ppp start filter solute %.1f, %4d/%02d/%02d %02d:%02d:%05.2f\n", d_tow, z_epochT.year, z_epochT.month,
       z_epochT.day, z_epochT.hour, z_epochT.min, z_epochT.second);
  if ((z_epochT.hour >= 7 && z_epochT.min >= 25 && z_epochT.second >= 0) || (d_tow >= 458306))
  {
    // printf("debug time\n");
  }
  /* reset filter of ppp */
  PPP_breakFilterOption(pz_satSigMeasCollect, &gz_pppAmbFixInputInfo, &gz_EKFstateRepPool, &gz_PPPfilterInfo);

  /* troposphere and ionosphere  empirical value */
  pp_UpdateIonosphericModel((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect);
  pp_UpdateTroposphericModel((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect);

  /* Cycle slip && receiver clock jump */
  cmn_gfDetectCycleSlip((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, &gz_gfDetectFilter);
  pp_ClkJumpRepaire(pz_SatSigMeasCollect_prev, (gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect);
  cmn_mwDetectCycleSlip((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, &gz_MwDetectFilter);
  pz_curDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  cmn_convertSigMeasToDcpStruct(pz_SsrLocBlk, pz_satSigMeasCollect, pz_curDcpMeas);
  z_tdcpMeasPointerPair.pz_curTdcpMeas = pz_curDcpMeas;
  z_tdcpMeasPointerPair.pz_preTdcpMeas = gpz_preTdcpMeasPPP;
  gz_PPPfilterInfo.u_tdcpMethodValid = cmn_detectCycleSlipByTDCP(z_tdcpMeasPointerPair, (gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect);
  gz_PPPfilterInfo.d_deltaTimeDopplerDetect = cmn_dopplerDetectCycleSlip((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, &gz_dopplerDetectPairingBlock);

  /* QR parity check*/
  gz_PPPfilterInfo.q_QRcheckStatus = FALSE;
  if (!ppp_QRcheck(pz_satSigMeasCollect, pz_SsrLocBlk, &gz_PPPfilterInfo, &gz_EKFstateRepPool))
  {
    gz_PPPfilterInfo.q_QRcheckStatus = TRUE;
  }

  /* ppp float solution */
  gz_PPPfilterInfo.q_filterPosValid = FALSE;
  q_status = PPP_filterSolute((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, pz_SsrLocBlk,
    &gz_PPPfilterInfo, &gz_EKFstateRepPool, pz_filterObs);

  *gpz_preTdcpMeasPPP = *pz_curDcpMeas;
  gpz_preTdcpMeasPPP->z_posSol = pz_satSigMeasCollect->z_positionFix;
  if (NULL != pz_curDcpMeas)
  {
    OS_FREE(pz_curDcpMeas);
  }

  /* ppp amb resolution */
  setAmbFixInputInfo(pz_satSigMeasCollect, pz_SsrLocBlk, &gz_PPPfilterInfo,
    &gz_EKFstateRepPool, pz_filterObs, &gz_pppAmbFixInputInfo);
  if (!q_status)
  {
    PPP_AmbResolution(&gz_pppAmbFixInputInfo, pz_fixedAmbPool);
  }

  /* position solution */
  ppp_CreatePosSolution(pz_satSigMeasCollect, &gz_PPPfilterInfo, pz_fixedAmbPool, pz_PositionFix, q_status);

  /* integrity */
  if(ppp_IntegValid())
  {
    /* log feature and PL */
    if (q_status == 0)
    {
      ppp_IntegSetCN040(pz_satSigMeasCollect);
      ppp_IntegSetPosStatus(pz_fixedAmbPool, pz_PositionFix->u_fixFlag);
      ppp_IntegSetPosAndPosRotation(pz_PositionFix->d_lla);
      ppp_IntegSaveFilterPositionMatrix(&gz_PPPfilterInfo, pz_fixedAmbPool, &gz_EKFstateRepPool); // get the QEcefPos (3x3)
      ppp_IntegCalSetDOPS(pz_satSigMeasCollect, pz_filterObs); // calculate Dop
      ppp_IntegFillMlFeature(pz_satSigMeasCollect, pz_PositionFix, &gz_PPPfilterInfo, &gz_EKFstateRepPool, pz_filterObs, &gz_pppAmbFixInputInfo, pz_fixedAmbPool);
      ppp_IntegSetMLScene(pz_satSigMeasCollect);
    }
    
    /* calculate */
    integ_PLResult_t z_pl = { 0 };
    ppp_IntegFillUpIntegrity(pz_PositionFix, &z_pl);
    ppp_AlgoSoltag(&gz_PPPfilterInfo, pz_PositionFix);
    gnss_log_ProtectionLevel(gz_PPPfilterInfo.u_solTag, &z_pl, pz_PositionFix);
    log_MachineLearingData();
    LOGI(TAG_PPP, "PL: %.2f, %.2f, %.2f, %d\n", z_pl.f_enu_protection_level[0], z_pl.f_enu_protection_level[1], z_pl.f_enu_protection_level[2], pz_PositionFix->u_fixFlag);
  }

  /* free */
  gnss_SatSigMeasCollect_Copy((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, pz_SatSigMeasCollect_prev);
  free_pz_FilterObs(&pz_filterObs);
  free_fixedSignalAmbPool(&pz_fixedAmbPool);
  pz_filterObs = NULL;
  pz_fixedAmbPool = NULL;
  gz_pppAmbFixInputInfo.pz_FilterObs = NULL;

  return gz_PPPfilterInfo.w_algStatus;
}