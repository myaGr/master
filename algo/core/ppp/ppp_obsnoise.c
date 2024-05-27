#include "ppp_obsnoise.h"
#include <mw_log.h>
#include "mw_alloc.h"
#include "../cmn/gnss_common.h"
#include "../cmn/cmn_CSmask_combine.h"

/**
 * @brief computation of pseudorange sigma
 * @param[in] pz_satSigMeasCollect is the gnss observation
 * @param[in] pz_obsCmcBlock is the previous cmc
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_FilterObs is the sigma of observation
 * @return 0 represent successful and other represent failure
 */
uint8_t ppp_pseudoSigmaCompute(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_filterInfo_t* pz_PPPfilterInfo, ppp_obsCmcBlock_t* pz_obsCmcBlock
  , ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_i = 0;
  uint8_t u_sat = 0;
  uint8_t u_freq = 0;
  uint8_t sig_num = 0;
  uint8_t u_isCurrentCloseSky = 0;
  char char_sat[4] = "";
  float   f_cmc = 0.0;
  double  d_wave = 0.0;
  ppp_freqCmc_t* pz_freqCmc = NULL;
  const gnss_SignalMeas_t* pz_signalMeas = NULL;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  for (u_sat = 0; u_sat < ALL_GNSS_SYS_SV_NUMBER; u_sat++)
  {
    if (NULL == pz_satSigMeasCollect->pz_satMeas[u_sat])
    {
      continue;
    }
    if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat])
    {
      pz_obsCmcBlock->pz_satCmcBlock[u_sat] = (ppp_satCmc_t*)OS_MALLOC_FAST(sizeof(ppp_satCmc_t));
      if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat])
      {
        continue;
      }
    }
    satidx_SatString(u_sat, char_sat);
    sig_num = PPP_signalNum(pz_satSigMeasCollect->pz_satMeas[u_sat],C_GNSS_OBS_TYPE_CR);
    for (u_freq = 0; u_freq < MAX_GNSS_SIGNAL_FREQ; u_freq++)
    {
      if (NULL == pz_satSigMeasCollect->pz_satMeas[u_sat]->pz_signalMeas[u_freq])
      {
        continue;
      }
      pz_signalMeas = pz_satSigMeasCollect->pz_satMeas[u_sat]->pz_signalMeas[u_freq];
      d_wave = wavelength(pz_signalMeas->u_signal);
      if (!pz_signalMeas->z_measStatusFlag.b_prValid || fabs(pz_signalMeas->d_carrierPhase) < FABS_ZEROS||d_wave==1.0)
      {
        continue;
      }
      if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq])
      {
        pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq]= (ppp_freqCmc_t*)OS_MALLOC_FAST(sizeof(ppp_freqCmc_t));
        if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq])
        {
          continue;
        }
        memset(pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq], 0, sizeof(ppp_freqCmc_t));
      }
      pz_freqCmc = pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq];
      pz_freqCmc->f_cmc = (float)(pz_signalMeas->d_pseudoRange - pz_signalMeas->d_carrierPhase * d_wave);
      if (1 == cmn_combineCycleSlipMask(pz_PPPfilterInfo->u_tdcpMethodValid, sig_num, pz_signalMeas->u_slipMask, pz_signalMeas->u_LLI
        , pz_PPPfilterInfo->d_deltaTimeDopplerDetect,u_isCurrentCloseSky, pz_signalMeas->u_slipMethodValid, NULL))
      {
        pz_freqCmc->f_meancmc = pz_freqCmc->f_cmc;
        LOGI(TAG_PPP, "Reset cmc sat=%s, freq=%d\n", char_sat, u_freq);
      }
      pz_freqCmc->z_time = pz_satSigMeasCollect->z_tor;
    }
  }

  for (u_sat = 0; u_sat < ALL_GNSS_SYS_SV_NUMBER; u_sat++)
  {
    if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat])
    {
      continue;
    }
    for (u_freq = 0; u_freq < MAX_GNSS_SIGNAL_FREQ; u_freq++)
    {
      if (NULL == pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq])
      {
        continue;
      }
      pz_freqCmc = pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq];
      if (tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_freqCmc->z_time) > 5.0)
      {
        OS_FREE(pz_obsCmcBlock->pz_satCmcBlock[u_sat]->pz_freqCmc[u_freq]);
        continue;
      }
      if (NULL == pz_FilterObs->pz_SatFilterObs[u_sat] || NULL == pz_FilterObs->pz_SatFilterObs[u_sat]->pf_codeSigma[u_freq])
      {
        continue;
      }
      *pz_FilterObs->pz_SatFilterObs[u_sat]->pf_codeSigma[u_freq] = (float)(fabs(pz_freqCmc->f_cmc - pz_freqCmc->f_meancmc) * 3.0);
      if (fabs(pz_freqCmc->f_cmc - pz_freqCmc->f_meancmc) > 3.0)
      {
        continue;
      }
      pz_freqCmc->f_meancmc = pz_freqCmc->f_meancmc * 0.5f + pz_freqCmc->f_cmc * 0.5f;
    }
  }

  return 0;
}