
#include <math.h>
#include "ppp_los_predict.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "ppp_type.h" 
#include "sd_api.h"

#define DATA_RESAMPLE_NUM           (DATA_MAX_HIS_TIME/SOL_INTERVAL_SEC+1)
#define SOL_LOSJUMP_THRESHOLD       (10)    /*unit:mm*/
#define MIN_EPOCH_LOS_HIS           (5)   /* min number of epoch for Predict */   
#define SOL_MAX_SSR_AGE_HIS         (30.0)

/**
 * @brief get the index of u_satidxlist
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    u_satidxlist is the list of  satellites
 * @return       the satellite index of u_satidxlist
 * */
uint8_t ppp_getsatLosIndex(gnss_ConstellationType u_constellation, uint8_t u_svid,uint8_t* u_satidxlist)
{
  uint8_t  u_i = 0;
  uint8_t  u_satidxlos = SSR_PREDICT_SAT_NUM_LIMIT;
  uint8_t  u_constellation_los= C_GNSS_NONE;

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (u_satidxlist[u_i] == 0)
    {
      u_satidxlos = u_i< u_satidxlos? u_i : u_satidxlos;
    }
    else if (u_svid == gnss_cvt_SvIndex2Svid(u_satidxlist[u_i]-1, &u_constellation_los) &&
      u_constellation == u_constellation_los)
    {
      u_satidxlos = u_i;
      break;
    }
  }
  return u_satidxlos;
}

/**
 * @brief  zero the corresponding u_svidx whose history los are all NULL
 * @param[in/out]    pz_LosPredictBlk is the lospredictBlk
 * @return           void 
 */
void ppp_satnumControl(ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (0 == pz_LosPredictBlk->u_satidxlist[u_i])
    {
      continue;
    }
    for (u_j = 0; u_j < pz_LosPredictBlk->w_nhis; u_j++)
    {
      if (NULL != pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_i])
      {
         break;
       }
    }
    if (u_j == pz_LosPredictBlk->w_nhis)
    {
      pz_LosPredictBlk->u_satidxlist[u_i] = 0;
      pz_LosPredictBlk->u_isNeedPredict[u_i / 8] = ~(1 << (u_i % 8)) & pz_LosPredictBlk->u_isNeedPredict[u_i / 8];//zero setting, do not predict
    }
  }
}
/**
* @brief store new los in losBlock
* @param[in]        pz_satSigMeasCollect the observation of GNSS
* @param[in]        pz_SsrLocBlk SSR block
* @param[in]        pq_basecorr, the value of base jump
* @param[in/out]    pz_LosPredictBlk provide the number of satellites and satellites Idx Table
* @param[out]       pz_epoch losBlock of current epoch
* @param[out]       pu_isNeedPredict is the flag of do predict or not
* return            the satillite count of currect epoch
*/
static uint8_t storeNewSsrLosBlock_t(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk
  , const int32_t* pq_basecorr,  ppp_LosPredictBlock_t* pz_LosPredictBlk,ppp_epochHisSSR_t* pz_epoch, uint8_t* pu_isNeedPredict)
{
  uint8_t  u_ns = 0;
  uint8_t  u_i = 0;
  uint8_t  u_sat= 0;
  uint32_t q_satidx = 0;
  double   d_age = 0.0;
  gnss_ConstellationType   u_constellation;
  gnss_Ephemeris_t pz_eph;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  // the bug is fixed in copyhisSsrBlock_t() and ppp_SsrInputLosPredict()!!
  memset(&pz_epoch->z_tor, 0, sizeof(GpsTime_t));
  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (NULL == pz_epoch->pz_satHisSSR[u_i])
    {
      continue;
    }
    OS_FREE(pz_epoch->pz_satHisSSR[u_i]);
    pz_epoch->pz_satHisSSR[u_i] = NULL;
  }  
 
  ppp_satnumControl(pz_LosPredictBlk);

  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    q_satidx = gnss_cvt_Svid2SvIndex(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation);
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satidx];
    if (NULL != pz_satMeas && pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 7.0)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid))
    {
      continue;
    }
    if (!(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
      || !(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || !(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    if (integrity_check(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.u_postItg,
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.u_preItg) >= 1.0)
    {
      continue;
    }
    d_age = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClkTime);
    if (fabs(d_age) > SOL_MAX_SSR_AGE_HIS)
    {
      continue;
    }
    u_constellation = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation;
    if (!sd_api_GnssEphemeris_Get(&pz_satSigMeasCollect->z_tor, u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid,
      pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].q_iode, &pz_eph))
    {
      continue;
    }
    // Satellite index matching
    u_sat = ppp_getsatLosIndex(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid, pz_LosPredictBlk->u_satidxlist);
    if (u_sat >= SSR_PREDICT_SAT_NUM_LIMIT)
    {
      continue;
    }
    else
    {
       if (!pz_LosPredictBlk->u_satidxlist[u_sat])
       {
         pz_LosPredictBlk->u_satidxlist[u_sat] = q_satidx + 1;
       }
    }
   
    if (NULL == pz_epoch->pz_satHisSSR[u_sat])
    {
      pz_epoch->pz_satHisSSR[u_sat] = (ppp_satHisSSR_t*)OS_MALLOC_FAST(sizeof(ppp_satHisSSR_t));
    }
    pz_epoch->pz_satHisSSR[u_sat]->z_orbtime = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClkTime;
    pz_epoch->pz_satHisSSR[u_sat]->q_los = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClk.q_corr;
    pz_epoch->pz_satHisSSR[u_sat]->q_los += pq_basecorr[u_constellation];
    pz_epoch->pz_satHisSSR[u_sat]->q_iode = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].q_iode;

    // it is need to predicted again 
   pu_isNeedPredict[u_sat / 8] = ~(1 << (u_sat % 8)) & pu_isNeedPredict[u_sat / 8];

    u_ns++;
  }
  if (u_ns > 0)
  {
    pz_epoch->z_tor = pz_SsrLocBlk->z_epochLosInfo.z_tor;
  }

  return u_ns;
}

/**
* @brief copy pz_src to pointer pz_dist
* @param[in]      pz_src is the source pointer
* @param[out]     pz_dist is the distination pointer
* return void
*/
 void copyhisSsrBlock_t(const ppp_epochHisSSR_t* pz_src, ppp_epochHisSSR_t* pz_dist)
{
  uint8_t u_i = 0;
  uint8_t u_satidx = 0;

  pz_dist->z_tor = pz_src->z_tor;
  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (NULL == pz_src->pz_satHisSSR[u_i])
    {
      if (NULL != pz_dist->pz_satHisSSR[u_i])
      {
        OS_FREE(pz_dist->pz_satHisSSR[u_i]);
      }
      continue;
    }
    if (NULL == pz_dist->pz_satHisSSR[u_i])
    {
      pz_dist->pz_satHisSSR[u_i] = (ppp_satHisSSR_t*)OS_MALLOC_FAST(sizeof(ppp_satHisSSR_t));
    }
    memcpy(pz_dist->pz_satHisSSR[u_i], pz_src->pz_satHisSSR[u_i], sizeof(ppp_satHisSSR_t));
  }
}

/**
* @brief check if the IODE of pz_SsrLocBlk and pz_LosPredictBlk match
* @param[in]     pz_SsrLocBlk is the SSR product
* @param[out]    pz_LosPredictBlk is LOS Predict infomation of SSR
* return void
*/
static void iodeCheck(const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sat = 0;
  uint8_t u_svid = 0;
  int32_t q_iode = -1;
  gnss_ConstellationType   u_constellation= C_GNSS_NONE;
  gnss_Ephemeris_t pz_eph;

  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    q_iode = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].q_iode;
    u_svid = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid;
    u_constellation = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation;
    if (!sd_api_GnssEphemeris_Get(&pz_SsrLocBlk->z_epochLosInfo.z_tor, u_constellation, u_svid, q_iode, &pz_eph))
    {
      continue;
    }
    u_sat = ppp_getsatLosIndex(u_constellation, u_svid, pz_LosPredictBlk->u_satidxlist);
    if (u_sat >= SSR_PREDICT_SAT_NUM_LIMIT)
    {
      continue;
    }
    for (u_j = 0; u_j < DATA_MAX_HIS_COUNT; u_j++)
    {
      if (NULL == pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat])
      {
        continue;
      }
      if (q_iode != pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat]->q_iode)
      {
        OS_FREE(pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat]);
        pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat] = NULL;
      }
    }
  }
}
/**
 * @brief check whether the new ssr can obtain the satellite ephemeris
*  @param[in]         pz_SsrLocBlk is the SSR product
 * @param[in/out]     pz_LosPredictBlk is LOS Predict infomation of SSR
 * @return            void
 */
static void newSSRHealthCheck(const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t u_i = 0;
  uint8_t u_sat = 0;
  uint8_t u_svid = 0;
  int32_t q_iode = -1;
  gnss_ConstellationType   u_constellation = C_GNSS_NONE;
  gnss_Ephemeris_t pz_eph;

  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    q_iode = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].q_iode;
    u_svid = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid;
    u_constellation = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation;
    u_sat = ppp_getsatLosIndex(u_constellation, u_svid, pz_LosPredictBlk->u_satidxlist);
    if (u_sat >= SSR_PREDICT_SAT_NUM_LIMIT)
    {
      continue;
    }
    if (!sd_api_GnssEphemeris_Get(&pz_SsrLocBlk->z_epochLosInfo.z_tor, u_constellation, u_svid, q_iode, &pz_eph))
    {
      pz_LosPredictBlk->u_isNeedPredict[u_sat / 8] = ~(1 << (u_sat % 8)) & pz_LosPredictBlk->u_isNeedPredict[u_sat / 8];
    }
  }
}

/**
 * @brief los of pz_SsrLocBlk input pz_LosPredictBlk
*  @param[in]      pz_satSigMeasCollect the observation of GNSS
*  @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_LosPredictBlk is LOS Predict infomation of SSR
 * @return         the number epoch of history ssr correction
 */
uint16_t ppp_SsrInputLosPredict(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t   u_j = 0;
  uint16_t  w_i = 0;
  uint16_t  w_epoch = 0;

  newSSRHealthCheck(pz_SsrLocBlk, pz_LosPredictBlk);

  if (pz_LosPredictBlk->w_nhis > 0 && tm_GpsTimeDiff(&pz_SsrLocBlk->z_epochLosInfo.z_tor,
    &pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis - 1].z_tor) < 4.999)   // more than 0.2 HZ
  {
    return 0; // the same ssr as previous
  }

  if (pz_LosPredictBlk->w_nhis > 0)// los jump, if iode change
  {
    iodeCheck(pz_SsrLocBlk, pz_LosPredictBlk);
  }
  /* store the the new gnss_ssrLosBlock_t */
  if (pz_LosPredictBlk->w_nhis < DATA_MAX_HIS_COUNT)
  {
    w_epoch = pz_LosPredictBlk->w_nhis;
    if (storeNewSsrLosBlock_t(pz_satSigMeasCollect, pz_SsrLocBlk, pz_LosPredictBlk->q_basecorr,pz_LosPredictBlk,
      &pz_LosPredictBlk->pz_epochHisSSR[w_epoch],pz_LosPredictBlk->u_isNeedPredict))
    {
      pz_LosPredictBlk->w_nhis++;
    }
  }
  else
  {
    /* delete the oldest ssr data */
    for (w_i = 0; w_i < DATA_MAX_HIS_COUNT - 1; w_i++)
    {
      copyhisSsrBlock_t(&pz_LosPredictBlk->pz_epochHisSSR[w_i + 1], &pz_LosPredictBlk->pz_epochHisSSR[w_i]);
    }
    if (!storeNewSsrLosBlock_t(pz_satSigMeasCollect, pz_SsrLocBlk, pz_LosPredictBlk->q_basecorr, pz_LosPredictBlk,
      &pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis - 1], pz_LosPredictBlk->u_isNeedPredict))
    {
      pz_LosPredictBlk->w_nhis--;
    }
  }
  if (pz_LosPredictBlk->w_nhis <= 0)
  {
    return 0;
  }

  /* time health checking */
  if (tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis - 1].z_tor,
    &pz_LosPredictBlk->pz_epochHisSSR[0].z_tor) < DATA_MAX_HIS_TIME)
  {
    return pz_LosPredictBlk->w_nhis; // the time of all data is health
  }

  for (w_i = 0; w_i < pz_LosPredictBlk->w_nhis; w_i++)
  {
    if (tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis - 1].z_tor,
      &pz_LosPredictBlk->pz_epochHisSSR[w_i].z_tor) < DATA_MAX_HIS_TIME)
    {
      break;
    }
    w_epoch = w_i + 1;
  }

  // remove the data of time out w_epoch
  for (w_i = 0; w_i < pz_LosPredictBlk->w_nhis - w_epoch; w_i++)
  {
    copyhisSsrBlock_t(&pz_LosPredictBlk->pz_epochHisSSR[w_i + w_epoch], &pz_LosPredictBlk->pz_epochHisSSR[w_i]);
  }
  // free the unused w_epoch memory
  for (w_i = pz_LosPredictBlk->w_nhis - w_epoch; w_i < pz_LosPredictBlk->w_nhis; w_i++)
  {
    if (w_i < 0|| w_i>=(DATA_MAX_HIS_COUNT)) continue;
    memset(& pz_LosPredictBlk->pz_epochHisSSR[w_i].z_tor, 0, sizeof(GpsTime_t));
    for (u_j = 0; u_j< SSR_PREDICT_SAT_NUM_LIMIT; u_j++)
    {
      if (NULL != pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_j])
      {
        OS_FREE(pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_j]);
      }
    }
  }
  pz_LosPredictBlk->w_nhis -= w_epoch;

  return pz_LosPredictBlk->w_nhis;
}


/**
 * @brief  identify the fiducial jump by the primary difference of history los of the target system
 * @param[in]     e_targetSys is the target sys
 * @param[in/out] pz_LosPredictBlk is LOS predict infomation of SSR
 * @return        1 represent have base-adj and 0 represent no base-adj
 */
uint8_t ppp_adjustSysBaseJump(gnss_ConstellationType e_targetSys, ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t   u_status = 0;
  uint8_t   u_sat = 0;
  uint8_t   u_jumpsatnum = 0;
  uint8_t   u_validsatnum = 0;
  uint16_t  w_j = 0;
  uint16_t  w_i = 0;
  uint16_t  w_validn = 0;
  int16_t   w_epoch =pz_LosPredictBlk->w_nhis - 1;
  int32_t   q_dlos = 0;
  int32_t   q_diff1=0;
  int32_t   q_basecorr = 0;
  double*   pd_diff = (double*)OS_MALLOC_FAST((pz_LosPredictBlk->w_nhis - 1) * sizeof(double));
  double    d_diffave = 0;
  double    d_diffstd = -1.0;
  double    d_threshold = 0;
  gnss_ConstellationType   u_constellation = C_GNSS_NONE;

  /*determine the benchmark jump time*/
  for (u_sat = 0; u_sat < SSR_PREDICT_SAT_NUM_LIMIT; u_sat++)
  {
    if (pz_LosPredictBlk->u_satidxlist[u_sat] <= 0)
    {
      continue;
    }
    gnss_cvt_SvIndex2Svid(pz_LosPredictBlk->u_satidxlist[u_sat]-1, &u_constellation);
    if (e_targetSys != u_constellation) continue;
    if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epoch - 1].pz_satHisSSR[u_sat]) continue;
    if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epoch].pz_satHisSSR[u_sat]) continue;
    u_validsatnum++;
    q_dlos = pz_LosPredictBlk->pz_epochHisSSR[w_epoch-1].pz_satHisSSR[u_sat]->q_los - pz_LosPredictBlk->pz_epochHisSSR[w_epoch].pz_satHisSSR[u_sat]->q_los;
    if (fabs(q_dlos) <= SOL_LOSJUMP_THRESHOLD)
    {
      break;
    }
    w_validn = 0;
    memset(pd_diff, 0, (pz_LosPredictBlk->w_nhis - 1) * sizeof(double));
    for (w_j = 1; w_j < pz_LosPredictBlk->w_nhis; w_j++)
    {
      if (NULL != pz_LosPredictBlk->pz_epochHisSSR[w_j].pz_satHisSSR[u_sat]&& NULL != pz_LosPredictBlk->pz_epochHisSSR[w_j-1].pz_satHisSSR[u_sat])
      {
        if (tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[w_j].z_tor, &pz_LosPredictBlk->pz_epochHisSSR[w_j - 1].z_tor) < 10)
        {
          pd_diff[w_validn] = 1.0*pz_LosPredictBlk->pz_epochHisSSR[w_j].pz_satHisSSR[u_sat]->q_los - pz_LosPredictBlk->pz_epochHisSSR[w_j-1].pz_satHisSSR[u_sat]->q_los;
          w_validn++;
        }
      }
    }
    gnss_ascSortMedianStdDouble(pd_diff, w_validn, &d_diffave, &d_diffstd);
    if( d_diffstd < 0 ) continue;
    d_threshold =3 * d_diffstd;
    if (fabs(q_dlos- d_diffave) > d_threshold)
    {
      u_jumpsatnum++;
      if (u_jumpsatnum>1&&fabs(q_dlos * 1.0 - q_diff1) > SOL_LOSJUMP_THRESHOLD)
      {
        u_jumpsatnum--;
        break;
      }
      q_diff1 = q_dlos;
      q_basecorr += q_diff1;
    }
    else break;
  }
  /*determine the correction for every vaild satllite*/
  if (u_jumpsatnum == u_validsatnum && u_jumpsatnum > 1)
  {
    pz_LosPredictBlk->q_basecorr[e_targetSys] = (int32_t)floor(q_basecorr * 1.0 / u_jumpsatnum + 0.5);
    LOGI(TAG_PPP, "LOS base jumpping sys=%d,jump value=%d\n", e_targetSys, pz_LosPredictBlk->q_basecorr[e_targetSys]);

    for (u_sat = 0; u_sat < SSR_PREDICT_SAT_NUM_LIMIT; u_sat++)
    {
      if (0 == pz_LosPredictBlk->u_satidxlist[u_sat])
      {
        continue;
      }
      gnss_cvt_SvIndex2Svid(pz_LosPredictBlk->u_satidxlist[u_sat]-1, &u_constellation);
      if (u_constellation == e_targetSys)
      {
        if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epoch].pz_satHisSSR[u_sat])
        {
          continue;
        }
        pz_LosPredictBlk->pz_epochHisSSR[w_epoch].pz_satHisSSR[u_sat]->q_los += pz_LosPredictBlk->q_basecorr[e_targetSys];
      }
    } 
    u_status = 1;
  }
  OS_FREE(pd_diff);
  pd_diff = NULL;
  return u_status;
}
/**
 * @brief cumulative sum of array
 * @param[in]      q_val is an array of size 1*n
 * @param[in]      w_n is the number of array
 * @param[out]     q_sum  is the cumulative sum of array(size 1*n)
 * @return         void
 */
 void ppp_cumsum(int32_t* q_var,uint16_t w_n,int32_t* q_sum) 
{
   uint16_t w_i = 0;
   q_sum[0] = q_var[0];
   for (w_i=1; w_i < w_n; w_i++)
   {
     q_sum[w_i] = q_sum[w_i-1]+q_var[w_i];
   }
}
 /**
 * @brief  Positive exponent criterion test of los for every satillite
 * @param[in]      pq_resampLos is resampled history los array of one satellite
 * @param[in]      u_satidx the index of pz_Los
 * @param[in]      w_resampcount is the number of  pq_resampLos array
 * @param[out]     pq_ago is the AGO list
 * @param[out]     q_positiveflag is the positive and exponential tests falg,1/-1 succeed, 0 failure;
 * @return         TRUE represent succeed and FALSE represent test failure
 */
 BOOL ppp_positiveExpoTest(int32_t* pq_resamplos,uint8_t u_satidx, uint16_t w_resampcount, int32_t* pq_ago, int8_t* positiveflag)
 {
   BOOL      w_state = FALSE;
   uint8_t   u_dcount1 = 0;
   uint8_t   u_dcount2 = 0;
   uint16_t  w_i = 0;
   uint16_t  w_j = 0;
   uint16_t  w_negacount = 0;
   uint16_t  w_posicount = 0;
   double    d_indictor1 = 0.0;
   double    d_indictor2 = 0.0;
   double*   pd_rho = NULL;
   char      c_sat[4] = "";
   satidx_SatString(u_satidx, c_sat);

   for (w_i = 0; w_i < w_resampcount; w_i++)
   {
     if (pq_resamplos[w_i] < 0)  w_negacount++;
     if (pq_resamplos[w_i] > 0)  w_posicount++;
   }
   if (w_resampcount == w_posicount)
   {
     *positiveflag = 1;
     w_state = TRUE;
   }
   else if (w_resampcount == w_negacount)
   {
     *positiveflag = -1;
     for (w_i = 0; w_i < w_resampcount; w_i ++)
     {
       pq_resamplos[w_i] = -1 * pq_resamplos[w_i];
     }
     w_state = TRUE;
   }
   else
   {
     *positiveflag = 0;
     w_state = FALSE;
     LOGI(TAG_PPP, "Los partial negative,sat=%s\n", c_sat);
   }
   /* exponent criterion test*/
   if (w_state)
   {
     pd_rho = (double*)OS_MALLOC_FAST((w_resampcount - 1) * sizeof(double));
     ppp_cumsum(pq_resamplos, w_resampcount, pq_ago);
     for (w_j = 0; w_j < w_resampcount - 1; w_j++)
     {
       pd_rho[w_j] = 1.0 * pq_resamplos[w_j + 1] / pq_ago[w_j];
       if (pd_rho[w_j] < 0.5)  u_dcount1++;
       if (pd_rho[w_j] < 0.5 && w_j>1) u_dcount2++;
     }
     d_indictor1 = 1.0 * u_dcount1 / (w_resampcount - 1);
     d_indictor2 = 1.0 * u_dcount2 / (w_resampcount - 3);
     if (d_indictor1 > 0.6 && d_indictor2 > 0.9)
     {
       w_state = TRUE;
     }
     else
     {
       w_state = FALSE;
       LOGI(TAG_PPP, "Los nonexponential trend,sat=%s\n", c_sat);
     }
   }
   if (NULL != pd_rho)
   {
     OS_FREE(pd_rho);
     pd_rho = NULL;
   }
   return w_state;
 }


/**
 * @brief  posteriori residuals test of the GM11 fitting 
 * @param[in]      pq_resampLos is resampled history los array of one satellite
 * @param[in]      q_testPredict is  test data for GM model calculations
 * @param[in]      w_resampcount the number of pq_resampLos array
 * @return         mean square error ratio
 */
double ppp_posterioRestest(int32_t* pq_resampLos, int32_t* q_testPredict, uint16_t w_resampcount)
{
  uint16_t w_i = 0;
  uint16_t w_num = 0;
  double   d_losmean = 0;
  double   d_losresmean = 0;
  double   d_sumlos = 0.0;
  double   d_sumlosres = 0.0;
  double   d_resratio = 99999.999;
  double*  pd_losres = NULL;
  double   d_sum = 0;
  double   q_std = 0;
  pd_losres = OS_MALLOC_FAST(w_resampcount * sizeof(double));

  for (w_i=0; w_i < w_resampcount; w_i++)
  {
    pd_losres[w_i] = (double)pq_resampLos[w_i];
  }
  gnss_ascSortMedianDouble(pd_losres, w_resampcount, &d_losmean);
  memset(pd_losres, 0, w_resampcount * sizeof(double));
  for (w_i=0; w_i < w_resampcount;w_i++)
  { 
    if (w_i > 0)
    {
        d_sumlos = d_sumlos + (1.0*pq_resampLos[w_i] - d_losmean) * (1.0 * pq_resampLos[w_i] - d_losmean);
    }
    if (q_testPredict[w_i] != 0 && pq_resampLos[w_i] != 0)
    {
      pd_losres[w_i] = 1.0 * q_testPredict[w_i] - 1.0 * pq_resampLos[w_i];
      d_sum = d_sum + pd_losres[w_i];
      w_num++;
    }
  }
  if (w_num < 3)
  {
    OS_FREE(pd_losres);
    pd_losres = NULL;
    return d_resratio;
  }
  d_losresmean = d_sum / w_num;
  for (w_i = 0; w_i < w_resampcount; w_i++)
  {
    if (q_testPredict[w_i] != 0 && pq_resampLos[w_i] != 0)
    {
      d_sumlosres = d_sumlosres + (pd_losres[w_i] - d_losresmean) *(pd_losres[w_i] - d_losresmean);
    }
  }
  if (d_sumlos > 2)
  {
    d_resratio = d_sumlosres / d_sumlos;/*>0.65 reject*/
  }
  OS_FREE(pd_losres);
  pd_losres = NULL;
  return d_resratio;
}
/**
 * @brief           calculated C by the weighted average of all components to build GM11
 * @param[in]       pq_resampLos is resampled history los array of one satellite
 * @param[in]       n is the size of pq_resample 
 * @param[in]       pq_ago is the first-order accumulating sequenc of pq_resampLos
 * @param[in]       pf_xindex  is the index of  resampled history los
 * @param[in]       a,b is the parameter of GM11 model
 * @return          c,the parameter of GM11 model     
 */
static double  ppp_calGmC(int32_t* pq_resampLos, uint16_t n, int32_t* pq_ago,float* pf_xindex, double a,double b)
{
  uint8_t  u_k = 0;
  uint16_t w_i = 0;
  double   d_beta = 0.0;
  double   d_sumeak = 0.0;
  double   d_sume2a = 0.0;
  double   d_sumk = 0.0;
  double   alaphx = 0.0;
  double   d_s = 0.0; 
  double   d_c = 0.0;

  for (u_k =0; u_k < n; u_k++)
  {
      d_sumk = d_sumk + pf_xindex[u_k];
  }
  for (u_k=0; u_k < n;u_k++)
  {
    d_sumeak= d_sumeak+ exp(-1 * a * pf_xindex[u_k])*pq_resampLos[u_k];
    d_sume2a = d_sume2a + exp(-2 * a * pf_xindex[u_k]);
    alaphx = alaphx + pf_xindex[u_k] / d_sumk * pq_ago[u_k];
  }
  d_s = (alaphx - b / a) * (1 - exp(a));
  d_beta = 1 / a * log(d_sumeak / (d_s * d_sume2a));

  d_c = (alaphx - b / a) * exp(a * d_beta);

  return d_c;
}
/**
 * @brief bulid GM11 and  forcast resample los for one satellite 
 * @param[in]       pq_resampLos is resampled history los array of one satellite
 * @param[in]       pq_ago is the first-order accumulating sequence
 * @param[in]       pf_xindex  is the index of  resampled history los
 * @param[in]       u_satidx the index of all satellites 
 * @param[in]       w_resampcount the number of pq_resampLos array
 * @param[in]       q_threshold is threshold of the first order difference 
 * @param[in]       w_predictn  is the tor corresponding to pf_xindex;
 * @param[out]      pq_predictLos is predicted values based on resampled history los array
 * @return          TRUE represent successs and FALSE represent failed
 */
BOOL ppp_bulidGMPredict(int32_t* pq_resampLos, int32_t* pq_ago, float* pf_xindex, uint8_t u_satidx, uint16_t w_resampcount
  , int32_t q_threshold, uint16_t w_predictn,int32_t *pq_predictLos)
{
  BOOL      state=TRUE;
  uint16_t  w_i = 0;
  int32_t   *pq_testPredict= NULL;
  int32_t   q_dlos = 0;
  int32_t   q_losend = 0;
  float     f_t = 0;
  double    *pq_nearmean= NULL;
  double    d_a = 0;
  double    d_b = 0;
  double    d_c = 0;
  double    d_resratio = 0.0;
  matrix_t* pz_y = matrix_new(w_resampcount-1,1);
  matrix_t* pz_B = matrix_new(w_resampcount - 1, 2);
  matrix_t* pz_Bt= matrix_new(2,w_resampcount - 1);
  matrix_t* pz_BtB = matrix_new(2,2);
  matrix_t* pz_BtBinv= matrix_new(2,2);
  matrix_t* pz_Bty= matrix_new(2, 1);
  matrix_t* pz_u= matrix_new(2, 1);
  char      c_sat[4] = "";
  pq_nearmean = OS_MALLOC_FAST((w_resampcount-1) * sizeof(double));
  pq_testPredict= OS_MALLOC_FAST(w_resampcount * sizeof(int32_t));

  for (w_i=0; w_i < w_resampcount - 1; w_i++)
  {
    pq_nearmean[w_i] =0.5*pq_ago[w_i] + 0.5*pq_ago[w_i + 1];
  }
  for (w_i = 0; w_i <w_resampcount-1; w_i++)
  {
    MAT(pz_y, w_i, 0) = pq_resampLos[w_i + 1];
    MAT(pz_B, w_i, 0) = -1.0 * pq_nearmean[w_i];
    MAT(pz_B, w_i, 1) = 1;
    MAT(pz_Bt, 0, w_i) = -1.0 * pq_nearmean[w_i];
    MAT(pz_Bt, 1, w_i) = 1;
  }
  matrix_mul(pz_Bt, pz_B,pz_BtB);
  if (matrix_inverse(pz_BtB, pz_BtBinv))
  {
    matrix_mul(pz_Bt, pz_y, pz_Bty);
    matrix_mul(pz_BtBinv, pz_Bty,pz_u);
    d_a = MAT(pz_u, 0, 0);
    d_b = MAT(pz_u, 1, 0);
    if (fabs(d_a) > FABS_ZEROS)
    {
      if (pq_resampLos[0] != 0)
      {
        d_c = (pq_ago[0] - d_b / d_a) * exp(d_a);
      }
      else if (pq_resampLos[w_resampcount - 1] != 0)
      {
        d_c = (pq_ago[w_resampcount - 1] - d_b / d_a) * exp(pf_xindex[w_resampcount-1] * d_a);
      }
      else
      {
        d_c = ppp_calGmC(pq_resampLos, w_resampcount, pq_ago, pf_xindex,d_a, d_b);
      }

      for (w_i = 0; w_i < w_resampcount ; w_i++)
      {
         pq_testPredict[w_i] = (int32_t)((1 - exp(d_a)) * d_c * exp(-1.0 * (w_i+1.0) * d_a));
        if (pq_resampLos[w_i] == 0) continue;
        q_losend = pq_resampLos[w_i];
        f_t = pf_xindex[w_resampcount - 1] - pf_xindex[w_i];
      }
      pq_predictLos[0] = (int32_t)((1 - exp(d_a)) * d_c * exp(-1.0 * (w_resampcount + 1.0) * d_a));
      pq_predictLos[1] = (int32_t)((1 - exp(d_a)) * d_c * exp(-1.0 * (w_resampcount + 1.0*w_predictn + 1.0) * d_a));

      /*GM Predict result check */
      d_resratio = ppp_posterioRestest(pq_resampLos, pq_testPredict, w_resampcount);
      q_dlos = q_losend - pq_predictLos[0];
      q_threshold = MAX_A_B(q_threshold, (10 + (int32_t)floor(5.0 * f_t + 0.5)));
      if (d_resratio > 2 || fabs(q_dlos) > q_threshold)
      {
        state = FALSE;
        satidx_SatString(u_satidx, c_sat);
        LOGI(TAG_PPP, "LOS GM11 fail,sat=%s,ratio=%.2f,los=%7d,%7d, dlos=%4d, thre1=%3d\n", c_sat, d_resratio, q_losend, pq_predictLos[1], q_dlos, q_threshold);
      }
    }
    else
    {
      state = FALSE;
    }
  }
  else
  {
    state = FALSE;
  }
  
  matrix_free(&pz_y);
  matrix_free(&pz_Bt);
  matrix_free(&pz_B);
  matrix_free(&pz_BtB);
  matrix_free(&pz_BtBinv);
  matrix_free(&pz_Bty);
  matrix_free(&pz_u);
  OS_FREE(pq_nearmean);
  pq_nearmean = NULL;
  OS_FREE(pq_testPredict);
  pq_testPredict = NULL;
  return state;
}
/**
 * @brief bulid GM11 and  forcast one satellite by resample los
 * @param[in]       pq_predictLos is predicted values based on resampled history los array
 * @param[in]       u_sat is the index of u_svIdx
 * @param[in]       q_positiveflag is the flag of positive of history los array
 * @param[in]       pz_LosPredict is the predicted los block
 * @param[out]      pz_PredictSSR is the SSR block
 * @return          void
 */
void ppp_resampRestore(int32_t* pq_predictLos, uint8_t u_sat,int8_t u_positiveflag, ppp_LosPredictBlock_t* pz_LosPredict
  , ppp_epochPredictSSR_t* pz_PredictSSR)
{
  uint16_t w_i = 0;
  int32_t  q_iode = -1;
  for (w_i = 0; w_i < DATA_MAX_HIS_COUNT; w_i++)
  {
    if (NULL == pz_LosPredict->pz_epochHisSSR[w_i].pz_satHisSSR[u_sat])
    {
      continue;
    }
    if (pz_LosPredict->pz_epochHisSSR[w_i].pz_satHisSSR[u_sat]->q_iode > 0)
    {
      q_iode = pz_LosPredict->pz_epochHisSSR[w_i].pz_satHisSSR[u_sat]->q_iode;
      break;
    }
  }
  if (NULL == pz_PredictSSR->pz_satPredictSSR[u_sat])
  {
    pz_PredictSSR->pz_satPredictSSR[u_sat] = (ppp_satPredictSSR_t*)OS_MALLOC_FAST(sizeof(ppp_satPredictSSR_t));
  }
  pz_PredictSSR->pz_satPredictSSR[u_sat]->q_iode = q_iode;
  pz_PredictSSR->pz_satPredictSSR[u_sat]->q_los = u_positiveflag * pq_predictLos[1];
  pz_PredictSSR->pz_satPredictSSR[u_sat]->u_losPredictFlag |= GNSS_LOS_PREDICT_FLAG_GM11;
}

/**
 * @brief t Calculate the threshold of the difference of adjacent resampled los
 * @param[in]         u_sat is the satellite index in u_svIdx
 * @param[in/out]     pz_LosPredictBlk is los Predict infomation of SSR
 * @return            q_diffthreshold the threshold of the difference of adjacent resampled los
 */
int32_t ppp_caldiffthreshold(uint8_t u_sat,ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint16_t w_epo = 0;
  uint16_t w_tempepo = 0;
  uint16_t w_i = 0;
  int16_t  w_nozeronum = 0;
  int32_t  q_sum = 0;
  int32_t* pq_diff = OS_MALLOC_FAST((DATA_MAX_HIS_COUNT - 1) * sizeof(int32_t));
  int32_t  q_diffthreshold = 0;
  double   d_sum = 0.0;
  double   d_mean = 0.0;
  double   d_std = 0.0;

  for (w_epo = 1; w_epo < DATA_MAX_HIS_COUNT; w_epo++)
  {
    if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_sat])
    {
      continue;
    }
    if (pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_sat]->q_los != 0)
    {
      w_tempepo= w_epo;
    }
    if (NULL != pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_sat] && NULL != pz_LosPredictBlk->pz_epochHisSSR[w_epo - 1].pz_satHisSSR[u_sat])
    {
      if (tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[w_epo].z_tor, &pz_LosPredictBlk->pz_epochHisSSR[w_epo - 1].z_tor) > 10) continue;
      if(pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_sat]->q_los == pz_LosPredictBlk->pz_epochHisSSR[w_epo - 1].pz_satHisSSR[u_sat]->q_los)  continue;
      pq_diff[w_nozeronum] = pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_sat]->q_los - pz_LosPredictBlk->pz_epochHisSSR[w_epo - 1].pz_satHisSSR[u_sat]->q_los;
      q_sum += pq_diff[w_nozeronum];
      w_nozeronum++;
    }
  }
  if (w_nozeronum == 0)
  {
    q_diffthreshold = 0;
  }
  else
  {
    d_mean = (1.0 * q_sum) / w_nozeronum;
    for (w_i = 0; w_i < w_nozeronum; w_i++)
    {
      d_sum = d_sum + (1.0 * pq_diff[w_i] - d_mean) * (1.0 * pq_diff[w_i] - d_mean);
    }
    if (d_sum > 0)
    {
      d_std = sqrt(d_sum / w_nozeronum + 0.05);
      q_diffthreshold = (int32_t)ceil(MAX_A_B(fabs(d_mean + 3 * d_std), fabs(d_mean - 3 * d_std)));
      pz_LosPredictBlk->pz_epochHisSSR[w_tempepo].pz_satHisSSR[u_sat]->q_diffthreshold = q_diffthreshold > 25 ? 0 : q_diffthreshold;
    }
    else
    {
      q_diffthreshold = 0;
    }
  }

  OS_FREE(pq_diff);
  pq_diff = NULL;
  return q_diffthreshold;
}

/**
 * @brief t Predict los by GM11 for one satellite
 * @param[in]         the tor of pz_satSigMeasCollect  
 * @param[in/out]     pz_LosPredictBlk is los predict infomation of SSR
 * @param[out]        pz_PredictSSR is result of predict infomation
 * @return             u_nsGM is the number of satellites successfully predict of GM11
 */
uint8_t ppp_LosPredictByGM11( const GpsTime_t* z_tor, ppp_LosPredictBlock_t* pz_LosPredictBlk, ppp_epochPredictSSR_t* pz_PredictSSR)
{
  int8_t    u_positiveflag = 0;/* 1/-1:all positive/negative; 0:part posi part nega; */
  uint8_t   w_resampcount = 0;
  uint8_t   u_i = 0;
  uint8_t   u_satidx = 0;
  uint8_t   u_n = 0;
  uint8_t   u_nsGM = 0;
  uint16_t  w_epo = 0;
  uint16_t  w_predictn = 0;
  int16_t   w_i = 0;
  int32_t   q_satlos = 0;
  int32_t   q_diffthreshold = 0;
  int32_t* pq_ago = NULL;
  int32_t* pq_resamplos = (int32_t*)OS_MALLOC_FAST(DATA_RESAMPLE_NUM * sizeof(int32_t));
  int32_t* pq_predictLos = (int32_t*)OS_MALLOC_FAST(2 * sizeof(int32_t));//predict los of first and current epoch
  float* pf_Xindex = (float*)OS_MALLOC_FAST(DATA_RESAMPLE_NUM * sizeof(float));
  double    d_dt = 0.0;
  char      c_sat[4] = "";
  GpsTime_t z_resampendtor;

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (!(pz_LosPredictBlk->u_isNeedPredict[u_i / 8] & (1 << (u_i % 8))))
    {
      continue;
    }
    if (!pz_LosPredictBlk->u_satidxlist[u_i])
    {
      continue;
    }
    u_satidx = pz_LosPredictBlk->u_satidxlist[u_i] - 1;
    q_satlos = 0; u_n = 0; w_resampcount = 0; q_diffthreshold = 0; w_predictn = 0;
    memset(pq_resamplos, 0, DATA_RESAMPLE_NUM * sizeof(int32_t));
    memset(pq_predictLos, 0,2* sizeof(int32_t));
    memset(pf_Xindex, 0, DATA_RESAMPLE_NUM * sizeof(float));
    memset(&z_resampendtor, 0, sizeof(GpsTime_t));

    for (w_epo = 0; w_epo < DATA_MAX_HIS_COUNT; w_epo++)
    {
      if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i])
      {
        continue;
      }
      if (abs(pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i]->q_los) <= FABS_ZEROS)
      {
        for (w_i = w_epo - 1; w_i < w_epo + 1; w_i++)
        {
          if (w_i < 0) continue;
          if (w_i >= DATA_MAX_HIS_COUNT) break;
          if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i]) continue;
          if (abs(pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i]->q_los) <= FABS_ZEROS)  continue;
          q_satlos += pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i]->q_los;
          u_n++;
        }
        if (u_n > 0)   q_satlos = (int32_t)floor(q_satlos * 1.0 / u_n + 0.5);
        if (q_satlos <= FABS_ZEROS)  continue;
        pq_resamplos[w_resampcount] = q_satlos;
      }
      else
      {
        pq_resamplos[w_resampcount] = pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i]->q_los;
      }
      pf_Xindex[w_resampcount] = w_resampcount + 1.0f;
      w_resampcount++;
      z_resampendtor = pz_LosPredictBlk->pz_epochHisSSR[w_epo].z_tor;
      d_dt = tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[w_epo].z_tor, &pz_LosPredictBlk->pz_epochHisSSR[0].z_tor);
    }
    if (w_resampcount < 5) continue;
    if (NULL != pq_ago)
    {
      OS_FREE(pq_ago);
      pq_ago = NULL;
    }
    pq_ago = (int32_t*)OS_MALLOC_FAST(w_resampcount * sizeof(int32_t));

    q_diffthreshold = ppp_caldiffthreshold(u_i, pz_LosPredictBlk);

    /*satnisfies the positive exponential law*/
    if (!ppp_positiveExpoTest(pq_resamplos, u_satidx, w_resampcount, pq_ago, &u_positiveflag))
    {
      continue;
    }
    
    //predict time corresponending to  pf_Xindex*/
    for (w_i = 0; w_i < (PPP_MAX_SSR_AGE/ SOL_INTERVAL_SEC); w_i++)
    {
      tm_GpstTimeAdd(&z_resampendtor, 1.0 * SOL_INTERVAL_SEC);
      if (tm_GpsTimeDiff(&z_resampendtor, z_tor) >= 0)
      {
        w_predictn = w_i;
        break;
      }
    }
    if (w_i == (PPP_MAX_SSR_AGE / SOL_INTERVAL_SEC))
    {
      continue;
    }
    /* determining model parameters and Predict*/
    if (!ppp_bulidGMPredict(pq_resamplos, pq_ago, pf_Xindex, u_satidx, w_resampcount, q_diffthreshold, w_predictn,pq_predictLos))
    {
      continue;
    }
    /* Resampled los reduction */
    ppp_resampRestore(pq_predictLos, u_i, u_positiveflag, pz_LosPredictBlk, pz_PredictSSR);
    satidx_SatString(u_satidx, c_sat);
    LOGD(TAG_PPP, "GM Predict:%s\n", c_sat);
    u_nsGM++;
  }

  OS_FREE(pq_predictLos);
  pq_predictLos = NULL;
  OS_FREE(pq_resamplos);
  pq_resamplos = NULL;
  if (NULL != pq_ago)
  {
    OS_FREE(pq_ago);
    pq_ago = NULL;
  }
  if (NULL != pf_Xindex)
  {
    OS_FREE(pf_Xindex);
    pf_Xindex = NULL;
  }

  return u_nsGM;
}
/**
 * @brief t Predict los by first-order polynomial fit for one satellite
 * @param[in]         the tor of pz_satSigMeasCollect
 * @param[in/out]     pz_LosPredictBlk is los Predict infomation of SSR
 * @param[out]        pz_PredictSSR is result of predict infomation
 * @return            u_gmsatcount the number of satellites successfully predict of first-order polynomial
 */
uint8_t ppp_LosPredictByPoly(const GpsTime_t *z_tor, ppp_LosPredictBlock_t* pz_LosPredictBlk, ppp_epochPredictSSR_t* pz_PredictSSR)
{
  uint8_t  u_i = 0;
  uint8_t  u_satidx = 0;
  uint8_t  u_nsPoly = 0;
  uint16_t w_epo = 0;
  uint16_t w_i = 0;
  int32_t  q_t = 0;
  int32_t  q_losend = 0;
  int32_t  q_dlos = 0;
  int32_t  q_threshold = 0;
  int32_t  q_dlosgm = 0;
  int32_t  q_dlospoly = 0;
  int32_t  q_satlos = 0;
  int32_t  q_iode = -1;
  uint32_t q_validcount = 0;
  double   d_sumx = 0;
  double   d_sumy = 0;
  double   d_sumxx = 0;
  double   d_sumxy = 0;
  double   d_b = 0.0;
  double   d_a = 0.0;
  double   d_x = 0.0;
  double   d_tempdx = 0.0;
  char c_sat[4] = "";
  GpsTime_t z_hisendtor;
  gnss_losPredictFlag  u_losflag;
  int32_t* q_orbclkPredict = OS_MALLOC_FAST(2 * sizeof(int32_t));

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (!(pz_LosPredictBlk->u_isNeedPredict[u_i / 8] & (1 << (u_i % 8))))
    {
      continue;
    }
    if (!pz_LosPredictBlk->u_satidxlist[u_i])
    {
      continue;
    }
    u_satidx = pz_LosPredictBlk->u_satidxlist[u_i] - 1;
    satidx_SatString(u_satidx, c_sat);
    d_sumx = 0; d_sumy = 0; q_t = 0; q_losend = 0; d_b = 0.0; d_a = 0.0;
    d_sumxx = 0; d_sumxy = 0; q_validcount = 0; q_dlospoly = 0;
    q_dlosgm = 0; u_losflag = GNSS_LOS_PREDICT_FLAG_FAIL;
    memset(&z_hisendtor, 0, sizeof(GpsTime_t));
    memset(q_orbclkPredict, 0, 2*sizeof(int32_t));

    for (w_epo = 0; w_epo < pz_LosPredictBlk->w_nhis; w_epo++)
    {
      if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i])
      {
        continue;
      }
      d_x = tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[w_epo].z_tor, &pz_LosPredictBlk->pz_epochHisSSR[0].z_tor) + SOL_INTERVAL_SEC;
      q_satlos = pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i]->q_los;
      if (q_satlos != 0)
      {
        d_sumx = d_sumx + d_x;
        d_sumy = d_sumy + q_satlos;
        d_sumxx = d_sumxx + d_x * d_x;
        d_sumxy = d_sumxy + d_x * q_satlos;
        q_validcount++;
        q_losend = q_satlos;
        q_t = pz_LosPredictBlk->w_nhis - w_epo;
        z_hisendtor = pz_LosPredictBlk->pz_epochHisSSR[w_epo].z_tor;
        q_threshold = pz_LosPredictBlk->pz_epochHisSSR[w_epo].pz_satHisSSR[u_i]->q_diffthreshold;
      }
    }
    if (q_validcount == 0) continue;
    d_b = (1.0 * q_validcount * d_sumxy - 1.0 * d_sumx * d_sumy) / (1.0 * q_validcount * d_sumxx - 1.0 * d_sumx * d_sumx);
    d_a = (d_sumy - d_b * d_sumx) / q_validcount;
    d_tempdx = tm_GpsTimeDiff(z_tor, &z_hisendtor);

    q_orbclkPredict[0] = (int32_t)floor(d_b * (d_x + 1.0 * SOL_INTERVAL_SEC) + d_a + 0.5);
    q_orbclkPredict[1] = (int32_t)floor(d_b * (d_x + d_tempdx) + d_a + 0.5);
    q_dlos = q_losend - q_orbclkPredict[0];
    q_threshold = MAX_A_B(q_threshold, (10 + 5 * (q_t - 1)));
    //GM11 compared Poly
    if (fabs(q_dlos) < q_threshold && q_threshold > 0)
    {
      if (NULL != pz_PredictSSR->pz_satPredictSSR[u_i]
        && (pz_PredictSSR->pz_satPredictSSR[u_i]->u_losPredictFlag & GNSS_LOS_PREDICT_FLAG_GM11))
      {
        q_dlosgm = q_losend - pz_PredictSSR->pz_satPredictSSR[u_i]->q_los;
        q_dlospoly = q_losend - q_orbclkPredict[1];
        if ((fabs(q_dlosgm) - fabs(q_dlospoly) > 5))
        {
          u_losflag = GNSS_LOS_PREDICT_FLAG_POLY;
        }
      }
      else
      {
        u_losflag = GNSS_LOS_PREDICT_FLAG_POLY;
      }
    }
    else
    {
      LOGI(TAG_PPP, "Poly Predict fail:%s %d %d %d %d\n", c_sat, q_losend, q_orbclkPredict[0], q_dlos, q_threshold);
    }

    if (u_losflag == GNSS_LOS_PREDICT_FLAG_POLY)
    {
      u_nsPoly++;
      LOGD(TAG_PPP, "Poly Predict:%s\n", c_sat);
      for (w_i = 0; w_i < DATA_MAX_HIS_COUNT; w_i++)
      {
        if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i])
        {
          continue;
        }
        if (pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i]->q_iode > 0)
        {
          q_iode = pz_LosPredictBlk->pz_epochHisSSR[w_i].pz_satHisSSR[u_i]->q_iode;
          break;
        }
      }
      if (NULL == pz_PredictSSR->pz_satPredictSSR[u_i])
      {
        pz_PredictSSR->pz_satPredictSSR[u_i] = (ppp_satPredictSSR_t*)OS_MALLOC_FAST(sizeof(ppp_satPredictSSR_t));
      }
      pz_PredictSSR->pz_satPredictSSR[u_i]->q_iode = q_iode;
      pz_PredictSSR->pz_satPredictSSR[u_i]->q_los = q_orbclkPredict[1];
      pz_PredictSSR->pz_satPredictSSR[u_i]->u_losPredictFlag = GNSS_LOS_PREDICT_FLAG_POLY;
    }
  }

  OS_FREE(q_orbclkPredict);
  q_orbclkPredict = NULL;

  return u_nsPoly;
}

/**
 * @brief trace to verify the correctness of the LOS prediction
 * @param[in]      pz_LosPredictBlk is LOS Predict infomation of SSR
 * @param[in]      u_satnum is the  number of satellites for the predicted LOS 
 * @param[in]      pz_PredictSSR is result of predict infomation
 * @return         void
 */
 void tracelosPredict( ppp_LosPredictBlock_t* pz_LosPredictBlk, uint8_t u_satnum,ppp_epochPredictSSR_t* pz_PredictSSR)
{
#if 0

  uint8_t     u_i = 0;
  char        c_satprn[4] = "";
  static FILE* fp = NULL;
  GpsTime_t   z_gpst;
  EpochTime_t z_curtime;

  if (NULL == fp )
  {
    fp = fopen("D:\\losPredict.ssr", "w");
  }
  if (fp == NULL) return;

  z_gpst = pz_LosPredictBlk->z_predictTime;
  tm_GpstTimeAdd(&z_gpst, 18.0);
  tm_cvt_GpstToEpoch(&z_gpst, &z_curtime);
  /* epoch time and ztd */
  fprintf(fp, "> Epoch %4d %02d %02d %02d %02d %04.1lf %3d     %5d \n", (int)z_curtime.year, (int)z_curtime.month, (int)z_curtime.day
    , (int)z_curtime.hour, (int)z_curtime.min, z_curtime.second, u_satnum, 1);

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (pz_LosPredictBlk->u_satidxlist[u_i] <= 0)
    {
      continue;
    }
    if (NULL != pz_PredictSSR->pz_satPredictSSR[u_i])
    {
      satidx_SatString(pz_LosPredictBlk->u_satidxlist[u_i]-1, c_satprn);
      fprintf(fp, "    %s", c_satprn);
    }
  }
  fprintf(fp, "\n");

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    if (NULL != pz_PredictSSR->pz_satPredictSSR[u_i])
    {
        fprintf(fp, "%7d", pz_PredictSSR->pz_satPredictSSR[u_i]->q_los);
    }
  }
 fprintf(fp, "\n");
 fclose(fp);

#endif // 0
}

/**
 * @brief decision of predict or not
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      z_tor is the tor of  pz_satSigMeasCollect 
 * @param[out]     pz_LosPredictBlk is LOS Predict infomation of SSR
 * @param[out]     pu_isNeedPredict is the flag of do predict or not
 * @return         other: do predict, 0: do not predict
 */
static uint8_t predictDecide(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  const GpsTime_t* z_tor, ppp_LosPredictBlock_t* pz_LosPredictBlk,uint8_t* pu_isNeedPredict)
{
  uint8_t u_flag = 0;
  uint8_t u_satidx = 0;
  uint8_t u_sat = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_n = 0;
  uint8_t u_noEph = 0;
  double  dt = 0.0;
  double  d_min_age = 999.9;
  gnss_Ephemeris_t pz_eph;
  ppp_satPredictSSR_t* pz_sat = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  if (pz_LosPredictBlk->w_nhis <= MIN_EPOCH_LOS_HIS)
  {
    return u_flag;
  }

  if (tm_GpsTimeDiff(z_tor, &pz_LosPredictBlk->z_predictTime) > 5.0)
  {
    memset(pz_LosPredictBlk->u_isNeedPredict, 0, sizeof(pz_LosPredictBlk->u_isNeedPredict));
  }
  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    pz_satLos = &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
    u_satidx = gnss_cvt_Svid2SvIndex(pz_satLos->u_svid, pz_satLos->u_constellation);
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satidx];
    if (NULL != pz_satMeas && pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 7.0)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satLos->u_constellation, pz_satLos->u_svid))
    {
      continue;
    }
    u_sat = ppp_getsatLosIndex(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid, pz_LosPredictBlk->u_satidxlist);
    if (u_sat >= SSR_PREDICT_SAT_NUM_LIMIT)
    {
      continue;
    }
    if (pz_LosPredictBlk->u_isNeedPredict[u_sat / 8] & (1 << (u_sat% 8)))
    {
      continue; // it has already been predicted
    }
    for (u_j = 0, u_n = 0; u_j < DATA_MAX_HIS_COUNT; u_j++)
    {
      if (NULL == pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat])
      {
        continue;
      }
      u_n++;
    }
    if (u_n < MIN_EPOCH_LOS_HIS)
    {
      continue;//Historical data is not enough to recommend prediction
    }
    if (!(tm_GpsTimeDiff(z_tor, &pz_satLos->z_orbClkTime) < SOL_MAX_SSR_AGE_HIS &&
      sd_api_GnssEphemeris_Get(z_tor, pz_satLos->u_constellation, pz_satLos->u_svid, pz_satLos->q_iode, &pz_eph)))
    {
      u_noEph++;
      pu_isNeedPredict[u_sat / 8] |= (1 << (u_sat % 8));
    }
  }
  if (u_noEph >= 2)  // there are more than two IODE check fail
  {
    u_flag = 1;
    return u_flag;
  }
  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].z_orbClkTime);
    if (dt < d_min_age)
    {
      d_min_age = dt;
    }
  }
  
  if (d_min_age > SOL_MAX_SSR_AGE_HIS)
  {
    for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
    {
      pz_satLos = &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
      u_satidx = gnss_cvt_Svid2SvIndex(pz_satLos->u_svid, pz_satLos->u_constellation);
      u_sat = ppp_getsatLosIndex(pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_constellation, pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i].u_svid, pz_LosPredictBlk->u_satidxlist);
      if (u_sat >= SSR_PREDICT_SAT_NUM_LIMIT)
      {
        continue;
      }
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satidx];
      if (NULL != pz_satMeas && pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 7.0)
      {
        continue;
      }
      if (C_SAT_TYPE_GEO == svid2SatType(pz_satLos->u_constellation, pz_satLos->u_svid))
      {
        continue;
      }
      for (u_j = 0, u_n = 0; u_j < DATA_MAX_HIS_COUNT; u_j++)
      {
        if (NULL == pz_LosPredictBlk->pz_epochHisSSR[u_j].pz_satHisSSR[u_sat])
        {
          continue;
        }
        u_n++;
      }
      if (u_n < MIN_EPOCH_LOS_HIS)
      {
        continue;//Historical data is not enough to recommend prediction
      }
      if (pz_LosPredictBlk->u_isNeedPredict[u_sat / 8] & (1 << (u_sat % 8)))
      {
        continue; // it has already been predicted 
      }

      pu_isNeedPredict[u_sat / 8] |= (1 << (u_sat% 8));
      u_flag = 2;
    }
    return u_flag; // the ssr break off
  }

  return u_flag;
}

/**
 * @brief inject the predicted los into pz_SsrLocBlk
 * @param[in]         z_tor is the tor of  pz_satSigMeasCollect 
 * @param[in]         pz_LosPredictBlk is LOS Predict infomation of SSR
 * @param[in/out]     pz_SsrLocBlk is the SSR product
 * @param[out]        pz_PredictSSR is result of predict infomation
 * @return         void
 */
static void losPredictToLosBlock(const GpsTime_t* z_tor, ppp_LosPredictBlock_t* pz_LosPredictBlk, gnss_ssrLosBlock_t* pz_SsrLocBlk
  , ppp_epochPredictSSR_t* pz_PredictSSR)
{
  uint8_t  u_i = 0;
  uint8_t  u_svid = 0;
  uint16_t w_nb = 0;
  uint16_t w_j = 0;
  GpsTime_t z_orbclktime = {0};
  gnss_ConstellationType u_constellation = C_GNSS_NONE;;
  gnss_Ephemeris_t pz_eph;
  gnss_satSsrLos_t* pz_satLos = NULL;
  ppp_satPredictSSR_t* pz_sat = NULL;
  char  c_sat[4] = "";
  char  c_buffer[256] = "";

  for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
  {
    pz_sat = NULL;
    if (!pz_LosPredictBlk->u_satidxlist[u_i])
    {
      continue;
    }
    if (NULL == pz_PredictSSR->pz_satPredictSSR[u_i])
    {
      continue;
    }
    if (pz_PredictSSR->pz_satPredictSSR[u_i]->u_losPredictFlag == GNSS_LOS_PREDICT_FLAG_FAIL)
    {
      continue;
    }
    if (tm_GpsTimeDiff(z_tor, &pz_LosPredictBlk->z_predictTime) <= 0.0)
    {
      pz_sat = pz_PredictSSR->pz_satPredictSSR[u_i];
    }
    if (NULL == pz_sat)
    { 
      continue;
    }
    u_svid = gnss_cvt_SvIndex2Svid(pz_LosPredictBlk->u_satidxlist[u_i]-1, &u_constellation);
    pz_satLos = getSSR_Los(u_constellation, u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos || (tm_GpsTimeDiff(z_tor, &pz_satLos->z_orbClkTime) < SOL_MAX_SSR_AGE_HIS &&
      sd_api_GnssEphemeris_Get(z_tor, u_constellation, u_svid, pz_satLos->q_iode, &pz_eph)))
    {
      continue;
    }
    for (w_j = DATA_MAX_HIS_COUNT; w_j >=1; w_j--)
    {
      if (NULL == pz_LosPredictBlk->pz_epochHisSSR[w_j-1].pz_satHisSSR[u_i])
      {
        continue;
      }
      z_orbclktime = pz_LosPredictBlk->pz_epochHisSSR[w_j - 1].pz_satHisSSR[u_i]->z_orbtime;
      break;
    }
    if (tm_GpsTimeDiff(z_tor, &z_orbclktime) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    pz_satLos->z_orbClkTime = z_orbclktime;
    pz_satLos->z_orbClk.q_corr = pz_sat->q_los - pz_LosPredictBlk->q_basecorr[u_constellation];
    if (pz_satLos->q_iode != pz_sat->q_iode)
    {
      pz_satLos->q_iode = pz_sat->q_iode;
      pz_satLos->u_orbClkMask &= ~(GNSS_SSR_SAT_ORB_CLK_EPH);
      memset(&pz_satLos->z_satPosVelClkBrdc, 0, sizeof(gnss_SatPosVelClk_t));
    }
    if (log_GetLogLevel() >= LOG_LEVEL_I&& w_nb<230)
    {
      svid_SatString(u_svid, u_constellation, c_sat);
      if (w_nb <= 0) w_nb += snprintf(&c_buffer[w_nb], 19, "los predict list:");
      w_nb += snprintf(&c_buffer[w_nb], 5, " %3s", c_sat);
    }
  }
  if (w_nb > 0 && log_GetLogLevel() >= LOG_LEVEL_I)
  {
    LOGI(TAG_PPP, "%s\n", c_buffer);
    strcpy(c_buffer, "");
    w_nb = 0;
  }
}
/**
 * @brief the interface of LOS Predict algorithm of SSR
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      z_tor is the tor of  pz_satSigMeasCollect 
 * @param[out]     pz_LosPredictBlk is LOS Predict infomation of SSR
 * @return         1 represent successs and other failed
 */
int32_t ppp_LosPredictSSR(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_ssrLosBlock_t* pz_SsrLocBlk
  , const GpsTime_t* z_tor , ppp_LosPredictBlock_t* pz_LosPredictBlk)
{
  uint8_t u_i = 0;
  uint8_t u_status = 0;
  uint8_t u_nsGM = 0;
  uint8_t u_nsPoly = 0;
  uint8_t u_isNeedPredict[SSR_PREDICT_SAT_NUM_LIMIT / 8 + 1] = {0};
  gnss_ConstellationType u_sys;
  if (NULL == pz_LosPredictBlk)  return 0;

  if (ppp_SsrInputLosPredict(pz_satSigMeasCollect,pz_SsrLocBlk, pz_LosPredictBlk))
  {
    /*correction of the base adjustment*/
    if (pz_LosPredictBlk->w_nhis> MIN_EPOCH_LOS_HIS && tm_GpsTimeDiff(&pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis-1].z_tor,
      &pz_LosPredictBlk->pz_epochHisSSR[pz_LosPredictBlk->w_nhis-2].z_tor) < 10)
    {
      /* benchmark adjustment*/
      for (u_sys = C_GNSS_GPS; u_sys < C_GNSS_MAX; u_sys++)
      {
        ppp_adjustSysBaseJump((gnss_ConstellationType)u_sys, pz_LosPredictBlk);
      }
    } 
  }

  if ((u_status=predictDecide(pz_satSigMeasCollect, pz_SsrLocBlk, z_tor, pz_LosPredictBlk, u_isNeedPredict)))
  {
    ppp_epochPredictSSR_t* pz_PredictSSR = (ppp_epochPredictSSR_t *)OS_MALLOC_FAST(sizeof(ppp_epochPredictSSR_t));
    memcpy(pz_LosPredictBlk->u_isNeedPredict, u_isNeedPredict, sizeof(u_isNeedPredict));
    /*GM(1,1) model prediction*/
    u_nsGM = ppp_LosPredictByGM11(z_tor, pz_LosPredictBlk, pz_PredictSSR);
    /*first-order polynomial fit prediction*/
    u_nsPoly = ppp_LosPredictByPoly(z_tor, pz_LosPredictBlk, pz_PredictSSR);
    if ((u_nsGM + u_nsPoly) > 0)
    {
      pz_LosPredictBlk->z_predictTime = *z_tor;
      losPredictToLosBlock(z_tor, pz_LosPredictBlk, pz_SsrLocBlk, pz_PredictSSR);
      //tracelosPredict(pz_LosPredictBlk, (u_nsGM + u_nsPoly),pz_PredictSSR);
    }
    
    LOGW(TAG_PPP, "los predict status=%d, GM: %d ,Poly: %d\n", u_status,u_nsGM, u_nsPoly);
    for (u_i = 0; u_i < SSR_PREDICT_SAT_NUM_LIMIT; u_i++)
    {
      if (NULL == pz_PredictSSR->pz_satPredictSSR[u_i])
      {
        continue;
      }
      OS_FREE(pz_PredictSSR->pz_satPredictSSR[u_i]);
    }
    OS_FREE(pz_PredictSSR);

  }

  
  
  return 1;
}

