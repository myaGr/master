/**@file        ppp_type.h
 * @brief       Location Engine PPP Structure Types
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/21  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "ppp_type.h"
#include "mw_log.h"
 /**
  * @brief get ssr data pointer
  * @param[in]    constellation
  * @param[in]    svid
  * @param[in]    signal
  * @param[in]    pz_ssrLocBlk
  * @return pointer
  */
extern const gnss_SignalCorr_t* getSSR_Bias(uint8_t constellation, uint8_t svid, uint8_t signal, const gnss_ssrLosBlock_t* pz_ssrLocBlk)
{
    uint8_t u_i = 0;
    const gnss_satSsrLos_t* pz_satLos = NULL;
    const gnss_SignalCorr_t* pz_signal = NULL;

    for (u_i = 0; u_i < pz_ssrLocBlk->z_epochLosInfo.u_satCount; u_i++)
    {
        if (svid == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid
            && constellation == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation)
        {
            pz_satLos = &pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
            break;
        }
    }
    if (NULL == pz_satLos)
    {
        return NULL;
    }
    for (u_i = 0; u_i < pz_satLos->u_signalNum; u_i++)
    {
        if (signal == pz_satLos->z_signalBiasCorr[u_i].u_signalType
            && (pz_satLos->z_signalBiasCorr[u_i].u_biasMask & GNSS_SSR_SAT_BIAS_CODE_CORR))
        {
            pz_signal = &pz_satLos->z_signalBiasCorr[u_i];
            break;
        }
    }
    return pz_signal;
}

/**
 * @brief get ssr data pointer
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    pz_ssrLocBlk
 * @return pointer
 */
extern const gnss_satSsrLos_t* getSSR_constLos(uint8_t constellation, uint8_t svid, const gnss_ssrLosBlock_t* pz_ssrLocBlk)
{
    uint8_t u_i = 0;
    const gnss_satSsrLos_t* pz_satLos = NULL;

    for (u_i = 0; u_i < pz_ssrLocBlk->z_epochLosInfo.u_satCount; u_i++)
    {
        if (svid == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid
            && constellation == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation)
        {
            pz_satLos = &pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
        }
    }
    if (NULL == pz_satLos)
    {
        return NULL;
    }

    return pz_satLos;
}

/**
 * @brief get ssr data pointer
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    pz_ssrLocBlk
 * @return pointer
 */
extern gnss_satSsrLos_t* getSSR_Los(uint8_t constellation, uint8_t svid, gnss_ssrLosBlock_t* pz_ssrLocBlk)
{
  uint8_t u_i = 0;
  gnss_satSsrLos_t* pz_satLos = NULL;

  for (u_i = 0; u_i < pz_ssrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    if (svid == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid
      && constellation == pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation)
    {
      pz_satLos = &pz_ssrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
    }
  }
  if (NULL == pz_satLos)
  {
    return NULL;
  }

  return pz_satLos;
}

/**
 * @brief sat meas`s singal num, filter NULL
 * @param[in] pz_satMeas
 * @param[in] u_obs_type
 * @return sat meas`s singal num 0~3
 */
extern uint8_t PPP_signalNum(const gnss_SatelliteMeas_t* pz_satMeas, gnss_ObsType u_obs_type)
{
  uint8_t n = 0;
  for (int i = 0; i < MAX_GNSS_SIGNAL_FREQ; ++i)
  {
    if (pz_satMeas->pz_signalMeas[i] != NULL)
    {
      if (C_GNSS_OBS_TYPE_PR == u_obs_type && fabs(pz_satMeas->pz_signalMeas[i]->d_pseudoRange) > FABS_ZEROS)
      {
        n++;
      }
      else if (C_GNSS_OBS_TYPE_CR == u_obs_type && fabs(pz_satMeas->pz_signalMeas[i]->d_carrierPhase) > FABS_ZEROS)
      {
        n++;
      }
    }
  }
  return n;
}

/**
 * @brief integrity of ssr checking
 * @param[in] post_itg
 * @param[in] pre_itg
 * @return health flag
 */
extern double integrity_check(gnss_integrityFlag post_itg, gnss_integrityFlag pre_itg)
{
#ifdef INTEGRITY_UNABLE
  return 0.001;
#endif // !INTEGRITY

  if ((post_itg == GNSS_INTEGRITY_FLAG_MONITORED_OK) && (pre_itg == GNSS_INTEGRITY_FLAG_MONITORED_OK))
  {
    return 1e-6;
  }
  else if ((post_itg == GNSS_INTEGRITY_FLAG_MONITORED_OK) && (pre_itg == GNSS_INTEGRITY_FLAG_NOT_MONITORED))
  {
    return 1e-3;
  }
  else if ((post_itg == GNSS_INTEGRITY_FLAG_NOT_MONITORED) && (pre_itg == GNSS_INTEGRITY_FLAG_MONITORED_OK))
  {
    return 1e-3;
  }
  LOGI(TAG_PPP, "Integrity is unhealth,post=%u, pre=%u\n", post_itg, pre_itg);
  return 1.0;
}