#include "rtk_type.h"
#include "gnss_common.h"
#include "sd_api.h"
#include "mw_log.h"
#include "mw_alloc.h"
/**
 * @brief get ssr data pointer
 * @param[in]    u_sys GNSS constellation
 * @param[in]    u_prn satellite index of the system
 * @param[in]    u_signalTargetType the signal type of observations
 * @param[in]    pz_rtkCorrBlock is the OSR correction
 * @return pointer of GnssMeas_t
 */
const GnssMeas_t* getRtkMeasCorr(uint8_t u_sys, uint8_t u_prn, gnss_SignalType u_signalTargetType, const GnssCorrBlock_t* pz_rtkCorrBlock)
{
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const GnssMeas_t* pz_corrMeas = NULL;
  uint16_t w_i = 0;
  gnss_FreqType z_targetFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  z_targetFreqType = gnss_cvt_Sig2FreqType(u_signalTargetType);
	if (z_targetFreqType != C_GNSS_FREQ_TYPE_MAX)
	{
		for (w_i = 0; w_i < (pz_rtkCorrBlock->w_measCount); ++w_i)
		{
			pz_corrMeas = &(pz_rtkCorrBlock->z_meas[w_i]);
			z_freqType = gnss_cvt_Sig2FreqType(pz_corrMeas->u_signal);
			if ((pz_corrMeas->u_constellation) == u_sys && (pz_corrMeas->u_svid) == u_prn && z_freqType == z_targetFreqType)
			{
				pz_corrMeasReslut = pz_corrMeas;
				break;
			}
		}
	}
  return pz_corrMeasReslut;
}
/**
 * @brief init the satellite position and clock of the Ref correct data
 * @param[in/out]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_initCorrSatPosClk(GnssCorrBlock_t* pz_GnssCorrBlockUpdate)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  for (u_i = 0; u_i < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_i)
  {
	pz_GnssCorrBlockUpdate->pz_satPosClk[u_i].u_svid = 0;
	pz_GnssCorrBlockUpdate->pz_satPosClk[u_i].u_constellation = 0;
	pz_GnssCorrBlockUpdate->pz_satPosClk[u_i].q_iode = -1;
	for (u_j = 0; u_j < 4; ++u_j)
	{
	  pz_GnssCorrBlockUpdate->pz_satPosClk[u_i].d_satPosClk[u_j] = 0.0;
	}
  }
  pz_GnssCorrBlockUpdate->w_satCount = 0;
  return;
}
/**
 * @brief get the index satellite position and clock of the Ref correct data
 * @param[in]    u_constellation GNSS constellation
 * @param[in]    u_svid satellite index of the system
 * @param[in]    pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[in]    w_satNum represent the satellite number of gnss_SatPosClkInfo_t struct
 * @return       the index satellite position and clock of the Ref correct data
 */
uint16_t rtk_getIndexInCorrSatPosClk(uint8_t u_constellation, uint8_t u_svid, const gnss_SatPosClkInfo_t pz_corrSatPosClk[MAX_GNSS_ACTIVE_SAT_NUMBER], uint16_t w_satNum)
{
  uint16_t w_iSat = 0;
  for (w_iSat = 0; w_iSat < w_satNum; ++w_iSat)
  {
	if (pz_corrSatPosClk[w_iSat].u_constellation == u_constellation && pz_corrSatPosClk[w_iSat].u_svid == u_svid)
	{
	  break;
	}
  }
  return w_iSat;
}
/**
 * @brief calculate the satellite position and clock of the Ref correct data
 * @param[in/out]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_calCorrDataSatPosClk(GnssCorrBlock_t* pz_GnssCorrBlockUpdate)
{
  uint16_t w_i = 0;
  uint16_t w_iSat = 0;
  uint8_t u_j = 0;
  GpsTime_t z_tot;
  gnss_SatPosVelClk_t z_SatPosVelClk = { 0 };
  const GnssMeas_t* pz_corrMeas = NULL;
  for (w_i = 0; w_i < (pz_GnssCorrBlockUpdate->w_measCount); ++w_i)
  {
	if ((pz_GnssCorrBlockUpdate->w_satCount) >= MAX_GNSS_ACTIVE_SAT_NUMBER)
	{
	  break;
	}
	pz_corrMeas = (const GnssMeas_t*)(&(pz_GnssCorrBlockUpdate->z_meas[w_i]));
	if ((pz_corrMeas->d_pseudoRange) <= 1.0e-2)
	{
	  continue;
	}
	w_iSat = rtk_getIndexInCorrSatPosClk(pz_corrMeas->u_constellation, pz_corrMeas->u_svid, pz_GnssCorrBlockUpdate->pz_satPosClk, pz_GnssCorrBlockUpdate->w_satCount);
	if (w_iSat < (pz_GnssCorrBlockUpdate->w_satCount))
	{
	  continue;
	}
	z_tot = pz_GnssCorrBlockUpdate->z_Clock.z_gpsTime;
	tm_GpstTimeAdd(&z_tot, -(pz_corrMeas->d_pseudoRange) / CLIGHT);
	if (!sd_api_SatPosVelClk_Get(pz_corrMeas->u_constellation, pz_corrMeas->u_svid, &z_tot, &z_SatPosVelClk))
	{
	  continue;
	}
	pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].u_constellation = (pz_corrMeas->u_constellation);
	pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].u_svid = (pz_corrMeas->u_svid);
	pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].q_iode = (z_SatPosVelClk.q_iode);
	for (u_j = 0; u_j < 4; ++u_j)
	{
	  pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].d_satPosClk[u_j] = z_SatPosVelClk.d_satPosClk[u_j];
	}
	++(pz_GnssCorrBlockUpdate->w_satCount);
  }
  return;
}
/**
 * @brief get the transfering time of correction data from satellite to site
 * @param[in]  u_constellation GNSS constellation
 * @param[in]  u_svid satellite index of the system
 * @param[out] pd_transferTime is the transfering time of Ref correct data
 * @return     TRUE represent successful, FALSE represent failure
 */
BOOL rtk_getCorrDataTransTime(uint8_t u_constellation, uint8_t u_svid, const GnssCorrBlock_t* pz_GnssCorrBlockUpdate, double* pd_transferTime)
{
  BOOL z_status = FALSE;
  uint16_t w_i = 0;
  const GnssMeas_t* pz_corrMeas = NULL;
  *pd_transferTime = 0.0;
  for (w_i = 0; w_i < (pz_GnssCorrBlockUpdate->w_measCount); ++w_i)
  {
	if ((pz_GnssCorrBlockUpdate->w_satCount) >= MAX_GNSS_ACTIVE_SAT_NUMBER)
	{
	  break;
	}
	pz_corrMeas = (const GnssMeas_t*)(&(pz_GnssCorrBlockUpdate->z_meas[w_i]));
	if ((pz_corrMeas->d_pseudoRange) <= 1.0e-2)
	{
	  continue;
	}
	if ((pz_corrMeas->u_constellation) == u_constellation && (pz_corrMeas->u_svid) == u_svid)
	{
	  *pd_transferTime = (pz_corrMeas->d_pseudoRange) / CLIGHT;
	  z_status = TRUE;
	  break;
	}
  }
  return z_status;
}
/**
 * @brief re-calculate the satellite position and satellite clock of correction data by matching the IODE of GNSS observation data
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_reCalculateCorrDataSatPosClkByRoverIode(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, GnssCorrBlock_t* pz_GnssCorrBlockUpdate)
{
  uint32_t q_satIndex = 0;
  uint16_t w_iSat = 0;
  uint8_t u_i = 0;
  GpsTime_t z_tot = { 0 };
  double d_transferTime = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatPosVelClk_t* pz_satPosVelClk = NULL;
  gnss_SatPosVelClk_t z_satPosVelClkResult = { 0 };
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
	pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
	if (NULL == pz_satMeas)
	{
	  continue;
	}
	pz_satPosVelClk = &(pz_satMeas->z_satPosVelClk);
	if ((pz_satPosVelClk->q_iode) < 0)
	{
	  continue;
	}
	w_iSat = rtk_getIndexInCorrSatPosClk(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_GnssCorrBlockUpdate->pz_satPosClk, pz_GnssCorrBlockUpdate->w_satCount);
	if (w_iSat >= (pz_GnssCorrBlockUpdate->w_satCount))
	{
	  continue;
	}
	
	if ((pz_satPosVelClk->q_iode) == (pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].q_iode))
	{
	  continue;
	}
	if (FALSE == rtk_getCorrDataTransTime(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_GnssCorrBlockUpdate, &d_transferTime))
	{
	  continue;
	}
	z_tot = pz_GnssCorrBlockUpdate->z_Clock.z_gpsTime;
	tm_GpstTimeAdd(&z_tot, -d_transferTime);
	if (FALSE == sd_api_SatPositionFromBrdc(&z_tot, pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satPosVelClk->q_iode, TRUE, &z_satPosVelClkResult))
	{
	  pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].q_iode = -1;
	  for (u_i = 0; u_i < 4; ++u_i)
	  {
		pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].d_satPosClk[u_i] = 0.0;
	  }
	  continue;
	}
	pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].q_iode = z_satPosVelClkResult.q_iode;
	for (u_i = 0; u_i < 4; ++u_i)
	{
		pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat].d_satPosClk[u_i] = z_satPosVelClkResult.d_satPosClk[u_i];
	}
  }
  return;
}
/**
 * @brief calculate the satellite number that the intersection of observations and VRS
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_OSRblock is the VRS observation information
 * @return     the satellite number that the intersection of observations and VRS
 */
uint8_t RTK_intersectionSatNumObsAndVRS(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_OSRblock)
{
	uint8_t u_satNum = 0;
	uint32_t q_satIndex = 0;
	uint8_t u_signalIndex = 0;
	uint16_t w_iSat = 0;
	BOOL z_isValid = FALSE;
	const gnss_SatelliteMeas_t* pz_satMeas = NULL;
	const gnss_SignalMeas_t* pz_sigMeas = NULL;
	const GnssMeas_t* pz_corrMeasReslut = NULL;
	for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
	{
		pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
		if (NULL == pz_satMeas)
		{
			continue;
		}
		z_isValid = FALSE;
		for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
		{
			pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
			if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
			{
				continue;
			}
			pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_OSRblock);
			if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
			{
				continue;
			}
			w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_OSRblock->pz_satPosClk, pz_OSRblock->w_satCount);
			if (w_iSat < (pz_OSRblock->w_satCount) && (pz_OSRblock->pz_satPosClk[w_iSat].q_iode) >= 0
				&& (pz_OSRblock->pz_satPosClk[w_iSat].q_iode) == (pz_satMeas->z_satPosVelClk.q_iode))
			{
				z_isValid = TRUE;
				break;
			}
		}
		if (TRUE == z_isValid)
		{
			++u_satNum;
		}
	}
	return u_satNum;
}
/**
 * @brief malloc the memory of struct rtk_EpochFilterObs_t
 * @param[out]  pz_satSigMeasCollect is the observation information
 * @return     the pointer of rtk_EpochFilterObs_t
 */
rtk_EpochFilterObs_t* malloc_pz_RTKfilterObs(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
	uint8_t u_i;
	uint8_t u_j;
	uint8_t u_status = 0;
	rtk_EpochFilterObs_t* pz_filterObs = NULL;

	pz_filterObs = (rtk_EpochFilterObs_t*)OS_MALLOC(sizeof(rtk_EpochFilterObs_t));

	if (NULL == pz_filterObs)
	{
		return NULL;
	}

	memcpy(&pz_filterObs->z_tor, &pz_satSigMeasCollect->z_tor,sizeof(GpsTime_t));

	pz_filterObs->w_numPhaseRes = 0;
	pz_filterObs->w_numOverQuarter = 0;
	pz_filterObs->f_pdop = 0.0;
	pz_filterObs->d_maxCodeBias = 0.0;
	pz_filterObs->d_maxPhaseBias = 0.0;
	pz_filterObs->d_priorCodeSTD = 0.0;
	pz_filterObs->d_priorPhaseSTD = 0.0;
	pz_filterObs->d_postCodeSTD = 0.0;
	pz_filterObs->d_postPhaseSTD = 0.0;
	pz_filterObs->d_prefixCodeSTD = 0.0;
	pz_filterObs->d_prefixPhaseSTD = 0.0;
	pz_filterObs->d_fixedCodeSTD = 0.0;
	pz_filterObs->d_fixedPhaseSTD = 0.0;
	pz_filterObs->d_rtdCodeSTD = 0.0;
	pz_filterObs->z_resType = 0;

	pz_filterObs->pz_MPflag = NULL;

	for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
	{
		if (NULL == pz_satSigMeasCollect->pz_satMeas[u_i])
		{
			pz_filterObs->pz_SatFilterObs[u_i] = NULL;
			continue;
		}

		pz_filterObs->pz_SatFilterObs[u_i] = (rtk_SatFilterObs_t*)OS_MALLOC(sizeof(rtk_SatFilterObs_t));
		if (NULL == pz_filterObs->pz_SatFilterObs[u_i])
		{
			LOGW(TAG_HPP, "%s pz_SatFilterObs malloc failed", __FUNCTION__);
			u_status = 1;
			break;
		}

		for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
		{
			if (NULL == pz_satSigMeasCollect->pz_satMeas[u_i]->pz_signalMeas[u_j])
			{
				pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j] = NULL;
				continue;
			}

			pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j] = (gnss_MeasStatusFlag_t*)OS_MALLOC(sizeof(gnss_MeasStatusFlag_t));
			if (NULL == pz_filterObs->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j])
			{
				LOGW(TAG_HPP, "%s z_measStatusFlag malloc failed", __FUNCTION__);
				u_status = 2;
				break;
			}

			/* init var */
			pz_filterObs->pz_SatFilterObs[u_i]->pf_codeResidual[u_j] = 0.0f;
			pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseResidual[u_j] = 0.0f;
			pz_filterObs->pz_SatFilterObs[u_i]->pf_codeVar[u_j] = 0.0f;
			pz_filterObs->pz_SatFilterObs[u_i]->pf_phaseVar[u_j] = 0.0f;
			pz_filterObs->pz_SatFilterObs[u_i]->pu_preFixUpdateFlag[u_j] = 0;
			pz_filterObs->pz_SatFilterObs[u_i]->pu_notNewLock[u_j] = 0;
		}
	}

	for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
	{
		for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
		{
			pz_filterObs->w_groupNUM[u_i][u_j] = 0;
			pz_filterObs->d_groupSTD[u_i][u_j] = 0.0;
		}
	}

	if (u_status)
	{
		free_pz_RTKFilterObs(&pz_filterObs);
		return NULL;
	}

	return pz_filterObs;
}
/**
 * @brief free pz_FilterObs
 * @param[in]  pz_FilterObs
 * @return     void
 */
void free_pz_RTKFilterObs(rtk_EpochFilterObs_t** pz_FilterObs)
{
	uint8_t u_i = 0;
	uint8_t u_j = 0;

	if (NULL == (*pz_FilterObs))
	{
		return;
	}

	if (NULL != (*pz_FilterObs)->pz_MPflag)
	{
		OS_FREE((*pz_FilterObs)->pz_MPflag);
	}

	for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
	{
		if (NULL == (*pz_FilterObs)->pz_SatFilterObs[u_i])
		{
			continue;
		}
		for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
		{

			if (NULL != (*pz_FilterObs)->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j])
			{
				OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]->z_measUpdateFlag[u_j]);
			}
		}
		OS_FREE((*pz_FilterObs)->pz_SatFilterObs[u_i]);
	}
	OS_FREE((*pz_FilterObs));
	(*pz_FilterObs) = NULL;
}