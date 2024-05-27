/**@file        swift_core_ssr_api.cpp
 * @brief       service ssr data of swift convert functions
 *              1. swift SSR data struct to Asensing SSR struct
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/20  <td>0.1      <td>liuguo      <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#if defined(APP_SWIFT_SSR)

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#ifdef __linux
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <direct.h>
#include <io.h>
#endif

#include <thread>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <cassert>
#include <iomanip>
#include <cmath>
#include <mutex>

#include "swift/config.h"
#include "swift/client.h"
#include "swift_core_ssr_api.h"


#define PI  (3.1415926535897932) /* pi */
#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */
#define MAX_LEN_SSRBUFF ((swift::rtcm_decoder::cRtcmMaxPayloadSize)*3)

typedef std::array<swift::SatellitePva, swift::cMaxSats> SatellitePvaArray;
typedef std::array<double, 3> RoverPosition;


static uint64_t ntrip_SSRBuffSum_len = 0u;
static uint8_t RT_ssr_stream_buff[1024*50]="";
static uint32_t gz_rt_len_ssr_buff=0;
static uint8_t pu_ssr_buff[MAX_LEN_SSRBUFF]="";
static uint32_t gz_len_ssr_buff=0;
static uint8_t payload[swift::rtcm_decoder::cRtcmMaxPayloadSize];
static uint32_t payload_length=0;
static uint32_t payload_i = 0;

static std::mutex Mutex_SwiftSDK;

struct SatelliteStates {
  uint32_t tow_ms = 0;
  uint16_t wn = 0;
  SatellitePvaArray sat_states{};
};

 // configuration structure
static swift::InitConfig swift_config;
// prepare callback structure
static swift::CallbackFunctions swift_callbacks{};
// create SsrClient object
static swift::SsrClient swift_client;
static SatelliteStates swift_sat_states;

struct SatelliteElevation {
  double elevation_rad{ 0.0 };
  // satellite constellation and number
  swift::SatelliteDescription sat_id;
};

typedef std::array<SatelliteElevation, swift::cMaxSats> SatelliteElevationArray;

static loc_api_gnssConstellationTypeVal constellationSwift2Ag(swift::GnssId id)
{
  loc_api_gnssConstellationTypeVal sys = LOC_API_GNSS_MAX;
  switch (id)
  {
  case swift::GnssId::GPS:
    sys = LOC_API_GNSS_GPS;
    break;
  case swift::GnssId::GAL:
    sys = LOC_API_GNSS_GAL;
    break;
  case swift::GnssId::BDS:
    sys = LOC_API_GNSS_BDS;
    break;
  default:
    break;
  }

  return sys;
}
static uint8_t SsrCorrSwift2Ag(uint8_t flag, const swift::SsrCorrection* swift_corr, loc_api_SsrCorrBias_t* ag_corr)
{
  if (!swift_corr->valid)
  {
    return 0;
  }
  ag_corr->q_corr = int32_t(swift_corr->corr * 1000 + 0.5);
  if (swift_corr->corr_std_dev.valid&& 0==flag)
  {
    ag_corr->f_qi = swift_corr->corr_std_dev.value * 0.25;
  }
  else if(1== flag)
  {
    ag_corr->f_qi =0.25 * 2.0*(swift_corr->bound_mean + 3.89 * swift_corr->bound_std_dev) / 3.89;  // 2 sigma/4
  }
  else
  {
    ag_corr->f_qi = 999.99f;
  }
  
  ag_corr->u_postItg = LOC_API_INTEGRITY_FLAG_MONITORED_OK;
  ag_corr->u_preItg = LOC_API_INTEGRITY_FLAG_MONITORED_OK;

  return 1;
}
static void signalTypeSwift2Ag_GPS(uint8_t swift_code, loc_api_SignalType* ag_code)
{
  *ag_code = LOC_API_GNSS_SIG_MAX;
  switch (swift_code)
  {
  case 0:
    *ag_code = LOC_API_GNSS_SIG_GPS_L1C;
    break;
  case 5:
    *ag_code = LOC_API_GNSS_SIG_GPS_L2C;
    break;
  case 15:
    *ag_code = LOC_API_GNSS_SIG_GPS_L5Q;
    break;
  default:
    break;
  }

  return;
}
static void signalTypeSwift2Ag_GAL(uint8_t swift_code, loc_api_SignalType* ag_code)
{
  *ag_code = LOC_API_GNSS_SIG_MAX;
  switch (swift_code)
  {
  case 2:
    *ag_code = LOC_API_GNSS_SIG_GAL_E1;
    break;
  case 6:
    *ag_code = LOC_API_GNSS_SIG_GAL_E5A;
    break;
  case 9:
    *ag_code = LOC_API_GNSS_SIG_GAL_E5B;
    break;
  default:
    break;
  }

  return;
}
static void signalTypeSwift2Ag_BDS(uint8_t swift_code, loc_api_SignalType* ag_code)
{
  *ag_code = LOC_API_GNSS_SIG_MAX;
  switch (swift_code)
  {
  case 0:
    *ag_code = LOC_API_GNSS_SIG_BDS_B1I;
    break;
  case 6:
    *ag_code = LOC_API_GNSS_SIG_BDS_B2I;
    break;
  case 3:
    *ag_code = LOC_API_GNSS_SIG_BDS_B3I;
    break;
  case 15:
    *ag_code = LOC_API_GNSS_SIG_BDS_B1C;
    break;
  case 13:
    *ag_code = LOC_API_GNSS_SIG_BDS_B2A;
    break;
  default:
    break;
  }

  return;
}
static void adjustWeekTow(uint16_t* week, double* tow)
{
	if (*tow < 0)
	{
	   *week -= 1;
	   *tow += 7.0 * 24 * 3600.0;
	}
	if (*tow >= 7.0 * 24 * 3600.0)
	{
	   *week += 1;
	   *tow -= 7.0 * 24 * 3600.0;
	}
}
/**
 * @brief Get the ionospheric coefficient base the L1 frequency
 * @param[in] constellation
 * @return the frequency of  base L1
 */
static double swift_BaseL1Freq(uint8_t constellation)
{
  const double FREQG1    =   1.57542E9 ;          /* L1/E1/B1C  frequency (Hz) */
  const double FREQR1    =   1.60200E9 ;          /* GLONASS G1 base frequency (Hz) */
  const double FREQC1    =   1.561098E9;          /* BDS B1I     frequency (Hz) */
  if (LOC_API_GNSS_GPS == constellation)
  {
    return FREQG1;
  }
  else if (LOC_API_GNSS_GLO == constellation)
  {
    return FREQR1;
  }
  else if (LOC_API_GNSS_BDS == constellation)
  {
    return FREQC1;
  }
  else if (LOC_API_GNSS_GAL == constellation)
  {
    return FREQG1;
  }
  else if (LOC_API_GNSS_QZS == constellation)
  {
    return FREQG1;
  }
  return 1.0;
}
static uint8_t satBlockSwift2Ag(const swift::SatelliteCorrection* swift_sat, loc_api_satSsrLos_t* ag_sat)
{
  static uint8_t u_clock_jump[LOC_API_GNSS_MAX][64] = { 0 };
  uint8_t u_i = 0;
  uint8_t u_status = 0;
  uint8_t u_SignalBiasCorrCount = 0;
  double d_age = 0.0;
  loc_api_gnssConstellationTypeVal u_constellation = constellationSwift2Ag(swift_sat->sv_desc.constellation);
  if (LOC_API_GNSS_MAX == u_constellation|| swift_sat->sv_desc.number<=0|| swift_sat->sv_desc.number>64)
  {
    return u_status;
  }
  if (swift_sat->flags & (1 << 12))
  {
    return u_status;
  }
  /*if (swift_sat->flags & (1 << 11))
  {
    if (u_clock_jump[u_constellation][swift_sat->sv_desc.number - 1] >= 255)
    {
      u_clock_jump[u_constellation][swift_sat->sv_desc.number - 1] = 0;
    }
    u_clock_jump[u_constellation][swift_sat->sv_desc.number - 1]++;
    
  }*/
  
  ag_sat->u_constellation=u_constellation;
  ag_sat->u_svid=swift_sat->sv_desc.number;
  ag_sat->d_windUp = 0.0;
  ag_sat->f_yaw = 0.0f;
  ag_sat->f_yawRate = 0.0f;
  ag_sat->u_windUpMask = LOC_API_SSR_ERROR_MODEL_SAT_WIND_UP_VALID;
  ag_sat->z_satPosVelClkBrdc.d_satPosClk[0] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satPosClk[1] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satPosClk[2] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satPosClk[3] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satVelClk[0] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satVelClk[1] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satVelClk[2] = 0.0;
  ag_sat->z_satPosVelClkBrdc.d_satVelClk[3] = 0.0;
  ag_sat->u_clockContinuityIod = u_clock_jump[u_constellation][swift_sat->sv_desc.number - 1];

  if (swift_sat->orbit.valid && swift_sat->clock.valid)
  {
    ag_sat->q_iode = -1;
    if (swift_sat->flags & (1 << 10))
    {
      ag_sat->q_iode = 0;
      for (u_i = 0; u_i < 10; u_i++)
      {
    	  if(swift_sat->flags & (1 << u_i))
    	  {
    		  ag_sat->q_iode += 1 << u_i;
    	  }
      }
    }
    printf("sys=%d id=%d, swift iode=%d\n",u_constellation,swift_sat->sv_desc.number,ag_sat->q_iode);
    //ag_sat->u_orbClkMask |= LOC_API_SSR_SAT_ORB_CLK_EPH;
    ag_sat->u_orbClkMask |= LOC_API_SSR_SAT_ORB_CLK_ORBIT_CORR;
    ag_sat->u_orbClkMask |= LOC_API_SSR_SAT_ORB_CLK_CLOCK_CORR;

    d_age = swift_sat->orbit.age_ms > swift_sat->clock.age_ms ? swift_sat->orbit.age_ms : swift_sat->clock.age_ms;
    ag_sat->z_orbClkTime.w_week = swift_sat->wn;
    ag_sat->z_orbClkTime.tow = (swift_sat->tow_ms - d_age) * 0.001;
    adjustWeekTow(&ag_sat->z_orbClkTime.w_week,&ag_sat->z_orbClkTime.tow);
    
    loc_api_SsrCorrBias_t orb = {0};
    loc_api_SsrCorrBias_t clk = {0};
    SsrCorrSwift2Ag(1,&swift_sat->orbit, &orb);
    SsrCorrSwift2Ag(1,&swift_sat->clock, &clk);

    ag_sat->z_orbClk = clk;
    ag_sat->z_orbClk.q_corr=-ag_sat->z_orbClk.q_corr-orb.q_corr;
    ag_sat->z_orbClk.f_qi = (float)(sqrt(orb.f_qi * orb.f_qi + clk.f_qi * clk.f_qi) * 1000.0);

    if (ag_sat->u_constellation == LOC_API_GNSS_GPS)
    {
      ag_sat->z_orbClk.f_qi *= 0.6f;
    }
    if (ag_sat->u_constellation == LOC_API_GNSS_GAL)
    {
      ag_sat->z_orbClk.f_qi *= 0.6f;
    }
    if (ag_sat->u_constellation == LOC_API_GNSS_BDS)
    {
      ag_sat->z_orbClk.f_qi *= 1.0f;
    }

    u_status = 1;
  }

  if (!(swift_sat->flags & (1 << 13)))
  {
    if (SsrCorrSwift2Ag(0,&swift_sat->iono, &ag_sat->z_stec))
    {
      double stec_fact = 40.31 * (1.0e+16) / (swift_BaseL1Freq(u_constellation)*swift_BaseL1Freq(u_constellation));
      ag_sat->z_stec.q_corr= int32_t(ag_sat->z_stec.q_corr/stec_fact + 0.5);
      ag_sat->z_stec.f_qi = ag_sat->z_stec.f_qi / stec_fact;
      ag_sat->u_atmoMask |= LOC_API_SSR_ATMO_STEC_CORR;
      ag_sat->z_STECtime.w_week= swift_sat->wn;
      ag_sat->z_STECtime.tow = (swift_sat->tow_ms- swift_sat->iono.age_ms) * 0.001;
      adjustWeekTow(&ag_sat->z_STECtime.w_week,&ag_sat->z_STECtime.tow);
    }
    if (SsrCorrSwift2Ag(1,&swift_sat->tropo, &ag_sat->z_std))
    {
      ag_sat->u_atmoMask |= LOC_API_SSR_ATMO_STD_CORR;
      ag_sat->z_STDtime.w_week = swift_sat->wn;
      ag_sat->z_STDtime.tow = (swift_sat->tow_ms- swift_sat->tropo.age_ms) * 0.001;
      adjustWeekTow(&ag_sat->z_STDtime.w_week,&ag_sat->z_STDtime.tow);
    }
    u_status = 1;
  }
  u_SignalBiasCorrCount = 0;
  for (auto code_it = swift_sat->code_bias.begin(); code_it != swift_sat->code_bias.end(); code_it++)
  {
    if (u_SignalBiasCorrCount >= LOC_API_SSR_SAT_CHL_NUM_LIMIT)
    {
      break;
    }
    if (!code_it->correction.valid)
    {
      continue;
    }
    loc_api_SignalType ag_code = LOC_API_GNSS_SIG_MAX;
    if (LOC_API_GNSS_GPS == u_constellation)
    {
      signalTypeSwift2Ag_GPS(code_it->signal_code, &ag_code);
    }
    else if (LOC_API_GNSS_GAL == u_constellation)
    {
      signalTypeSwift2Ag_GAL(code_it->signal_code, &ag_code);
    }
    else if (LOC_API_GNSS_BDS == u_constellation)
    {
      signalTypeSwift2Ag_BDS(code_it->signal_code, &ag_code);
    }
    if (ag_code == LOC_API_GNSS_SIG_MAX)
    {
      continue;
    }
    loc_api_SignalCorr_t* p_dstSignalCorr = &(ag_sat->z_signalBiasCorr[u_SignalBiasCorrCount]);
    if (!SsrCorrSwift2Ag(1,&code_it->correction, &p_dstSignalCorr->z_codeBias))
    {
      continue;
    }
    p_dstSignalCorr->u_biasMask |= LOC_API_SSR_SAT_BIAS_CODE_CORR;
    p_dstSignalCorr->u_signalType = ag_code;
    u_SignalBiasCorrCount++;
    u_status = 1;
  }

  for (auto phase_it = swift_sat->phase_bias.begin(); phase_it != swift_sat->phase_bias.end(); phase_it++)
  {
    if (!phase_it->correction.valid)
    {
      continue;
    }
    loc_api_SignalType ag_code = LOC_API_GNSS_SIG_MAX;
    if (LOC_API_GNSS_GPS == u_constellation)
    {
      signalTypeSwift2Ag_GPS(phase_it->signal_code, &ag_code);
    }
    else if (LOC_API_GNSS_GAL == u_constellation)
    {
      signalTypeSwift2Ag_GAL(phase_it->signal_code, &ag_code);
    }
    else if (LOC_API_GNSS_BDS == u_constellation)
    {
      signalTypeSwift2Ag_BDS(phase_it->signal_code, &ag_code);
    }
    if (ag_code == LOC_API_GNSS_SIG_MAX)
    {
      continue;
    }
    for (u_i = 0; u_i < u_SignalBiasCorrCount; u_i++)
    {
      if (ag_code == ag_sat->z_signalBiasCorr[u_i].u_signalType)
      {
        break;
      }
    }
    loc_api_SignalCorr_t* p_dstSignalCorr = NULL;
    if (u_i < u_SignalBiasCorrCount)
    {
      p_dstSignalCorr = &(ag_sat->z_signalBiasCorr[u_i]);
    }
    else if (u_SignalBiasCorrCount < LOC_API_SSR_SAT_CHL_NUM_LIMIT)
    {
      p_dstSignalCorr = &(ag_sat->z_signalBiasCorr[u_SignalBiasCorrCount]);
      u_SignalBiasCorrCount++;
    }
    else
    {
      continue;
    }
    if (!SsrCorrSwift2Ag(1,&phase_it->correction, &p_dstSignalCorr->z_phaseBias))
    {
      continue;
    }
    p_dstSignalCorr->u_biasMask |= LOC_API_SSR_SAT_BIAS_PHASE_CORR;
    p_dstSignalCorr->u_signalType = ag_code;
    p_dstSignalCorr->u_discontinuityIod = 0;
    u_status = 1;
  }

  ag_sat->u_signalNum = u_SignalBiasCorrCount;
  return u_status = 1;
}
 /**
   * @brief Convert SSR los from swift to asensing
   * @param[in] rover_position - position of rover
   * @param[in] pz_swift_los - swift SSR los data
   * @param[in] pz_SsrLosBlk - Asensing SSR los data
   * @return    satellite number
   */
static uint8_t ssr_cvt_SsrLosBlk_Swift2Ag(uint16_t w_week, uint32_t w_tow, const std::array<double, 3>& rover_position, const swift::LosCorrections* pz_swift_los,
  loc_api_ssrLosBlock_t* pz_SsrLosBlk)
{

  uint8_t u_sat_num = 0;
  if (NULL == pz_swift_los || NULL == pz_SsrLosBlk|| pz_swift_los->sat_correction_block.size()<=0)
  {
    return 0;
  }
  pz_SsrLosBlk->w_size = sizeof(loc_api_ssrLosBlock_t);
  pz_SsrLosBlk->d_siteCoor[0] = rover_position[0];
  pz_SsrLosBlk->d_siteCoor[1] = rover_position[1];
  pz_SsrLosBlk->d_siteCoor[2] = rover_position[2];

  loc_api_epochSsrLos_t* pz_ApiEpochSsrLos = &(pz_SsrLosBlk->z_epochLosInfo);

  pz_ApiEpochSsrLos->z_tor.w_week = w_week;
  pz_ApiEpochSsrLos->z_tor.tow = w_tow;
  pz_ApiEpochSsrLos->z_ZTDtime.w_week = 0;
  pz_ApiEpochSsrLos->z_ZTDtime.tow = 0.0;
  pz_ApiEpochSsrLos->u_ZTDmask = 0;
  pz_ApiEpochSsrLos->u_errorModelMask = 0;
  pz_ApiEpochSsrLos->z_ZTDcorr.f_qi = 999.99f;
  pz_ApiEpochSsrLos->z_ZTDcorr.d_dryCorr = 0.0;
  pz_ApiEpochSsrLos->z_ZTDcorr.d_wetCorr = 0.0;
  pz_ApiEpochSsrLos->z_ZTDcorr.u_postItg = LOC_API_INTEGRITY_FLAG_NOT_MONITORED;
  pz_ApiEpochSsrLos->z_ZTDcorr.u_preItg = LOC_API_INTEGRITY_FLAG_NOT_MONITORED;
  pz_ApiEpochSsrLos->u_satCount = 0;

  for (auto it = pz_swift_los->sat_correction_block.begin(); it != pz_swift_los->sat_correction_block.end(); it++)
  {
    if (!it->valid || it->sv_desc.number == 0) 
    {
      continue;
    }
    if (u_sat_num>=LOC_API_SSR_SAT_NUM_LIMIT)
    {
      break;
    }
    if (satBlockSwift2Ag(&(*it), &pz_ApiEpochSsrLos->z_satLosCorr[u_sat_num]))
    {
      u_sat_num++;
    }
  }
  pz_ApiEpochSsrLos->u_satCount=u_sat_num;
  printf("Number of AG-SSR:%d\n",u_sat_num);
  return u_sat_num;
}

static void SwiftGPSEphToApi(const swift::EphemerisGps* swiftEph, loc_api_GpsEphemeris_t* locApieph)
{
  locApieph->svid = swiftEph->sat_id;
  locApieph->week = swiftEph->wn;
  locApieph->accuracy = swiftEph->ura;
  locApieph->health = swiftEph->health;
  locApieph->iodc = swiftEph->iodc;
  locApieph->code = swiftEph->code_l2;
  locApieph->fit = swiftEph->fit_interval;
  locApieph->iode = swiftEph->iode;

  locApieph->tgd = swiftEph->tgd;
  locApieph->toc = swiftEph->toc;
  locApieph->toe = swiftEph->toe;
  locApieph->sqrt_A = swiftEph->sqrt_a;
  locApieph->e = swiftEph->ecc;
  locApieph->M0 = swiftEph->m0;
  locApieph->Omega0 = swiftEph->omega0;
  locApieph->Omega = swiftEph->omega;
  locApieph->OmegaDot = swiftEph->omega_dot;

  locApieph->i0 = swiftEph->i0;
  locApieph->idot = swiftEph->idot;
  locApieph->DeltaN = swiftEph->dn;
  locApieph->crc = swiftEph->crc;
  locApieph->crs = swiftEph->crs;
  locApieph->cuc = swiftEph->cuc;
  locApieph->cus = swiftEph->cus;
  locApieph->cic = swiftEph->cic;
  locApieph->cis = swiftEph->cis;
  locApieph->af0 = swiftEph->af0;
  locApieph->af1 = swiftEph->af1;
  locApieph->af2 = swiftEph->af2;
}

static void SwiftBDSEphToApi(const swift::EphemerisBds* swiftEph, loc_api_BdsEphemeris_t* locApieph)
{
  locApieph->svid = swiftEph->sat_id;
  locApieph->week = swiftEph->wn;
  locApieph->accuracy = swiftEph->urai;
  locApieph->health = swiftEph->health;
  locApieph->iodc = swiftEph->aodc;
  locApieph->code = 0;
  locApieph->fit = 0;
  locApieph->iode = swiftEph->aode;

  locApieph->tgdB1I = swiftEph->tgd1;
  locApieph->tgdB2I = swiftEph->tgd2;
  locApieph->toc = swiftEph->toc;
  locApieph->toe = swiftEph->toe;
  locApieph->sqrt_A = swiftEph->sqrt_a;
  locApieph->e = swiftEph->ecc;
  locApieph->M0 = swiftEph->m0;
  locApieph->Omega0 = swiftEph->omega0;
  locApieph->Omega = swiftEph->omega;
  locApieph->OmegaDot = swiftEph->omega_dot;

  locApieph->i0 = swiftEph->i0;
  locApieph->idot = swiftEph->idot;
  locApieph->DeltaN = swiftEph->dn;
  locApieph->crc = swiftEph->crc;
  locApieph->crs = swiftEph->crs;
  locApieph->cuc = swiftEph->cuc;
  locApieph->cus = swiftEph->cus;
  locApieph->cic = swiftEph->cic;
  locApieph->cis = swiftEph->cis;
  locApieph->af0 = swiftEph->af0;
  locApieph->af1 = swiftEph->af1;
  locApieph->af2 = swiftEph->af2;
}
static void SwiftGALEphToApi(const swift::EphemerisGal* swiftEph, loc_api_GalEphemeris_t* api_eph)
{
  api_eph->svid = swiftEph->sat_id;
  api_eph->week = swiftEph->wn;
  api_eph->accuracy = swiftEph->sisa_e1e5b;
  api_eph->E1health = swiftEph->sh_e1b;
  api_eph->E1DVS = swiftEph->dv_e1b;
  api_eph->E5bhealth = swiftEph->sh_e5b;
  api_eph->E5bDVS = swiftEph->dv_e5b;
  api_eph->iodc = 0;
  api_eph->code = 0;
  api_eph->fit = 0;
  api_eph->iode = swiftEph->iodn;

  api_eph->tgdE1E5a = swiftEph->bgd_e5a_e1;
  api_eph->tgdE1E5b = swiftEph->bgd_e5b_e1;
  api_eph->toc = swiftEph->toc;
  api_eph->toe = swiftEph->toe;
  api_eph->sqrt_A = swiftEph->sqrt_a;
  api_eph->e = swiftEph->ecc;
  api_eph->M0 = swiftEph->m0;
  api_eph->Omega0 = swiftEph->omega0;
  api_eph->Omega = swiftEph->omega;
  api_eph->OmegaDot = swiftEph->omega_dot;

  api_eph->i0 = swiftEph->i0;
  api_eph->idot = swiftEph->idot;
  api_eph->DeltaN = swiftEph->dn;
  api_eph->crc = swiftEph->crc;
  api_eph->crs = swiftEph->crs;
  api_eph->cuc = swiftEph->cuc;
  api_eph->cus = swiftEph->cus;
  api_eph->cic = swiftEph->cic;
  api_eph->cis = swiftEph->cis;
  api_eph->af0 = swiftEph->af0;
  api_eph->af1 = swiftEph->af1;
  api_eph->af2 = swiftEph->af2;
}
//static FILE* fp_EPH=fopen("/home/root/data/eph.log","w");
static void decoder_callback_GPSeph_handle(const swift::EphemerisGps &gps_eph_msg)
{
	//fprintf(fp_EPH,"Swift Eph: G%02d iode=%d  %4d %d\n",gps_eph_msg.sat_id,gps_eph_msg.iode,gps_eph_msg.wn,gps_eph_msg.toe);

	loc_api_GnssEphemeris_t loc_api_eph={0};
    SwiftGPSEphToApi(&gps_eph_msg, &loc_api_eph.eph.z_gpsEph);

	loc_api_eph.u_version=VERSION_GNSS_EPHEMERIS;
	loc_api_eph.u_constellation = LOC_API_GNSS_GPS;
	loc_api_eph.q_towMsecOfToe = (uint32_t)(gps_eph_msg.toe*1000.0+0.5);
	loc_api_eph.w_weekOfToe = gps_eph_msg.wn;
	loc_api_eph.q_towMsecOfToc = (uint32_t)(gps_eph_msg.toc*1000.0+0.5);
	loc_api_eph.w_weekOfToc = gps_eph_msg.wn;

	loc_api_InjectApiEphemeris((uint8_t*)(&loc_api_eph), sizeof(loc_api_GnssEphemeris_t));
}
static void decoder_callback_GALeph_handle(const swift::EphemerisGal &gal_eph_msg)
{
	uint16_t week=0;
	double tow=0.0;
	//fprintf(fp_EPH,"Swift Eph: E%02d iode=%d  %4d %d\n",gal_eph_msg.sat_id,gal_eph_msg.iodn, gal_eph_msg.wn,gal_eph_msg.toe);

	loc_api_GnssEphemeris_t loc_api_eph={0};
    SwiftGALEphToApi(&gal_eph_msg, &loc_api_eph.eph.z_galEph);

	loc_api_eph.u_version=VERSION_GNSS_EPHEMERIS;
	loc_api_eph.u_constellation = LOC_API_GNSS_GAL;
	week=gal_eph_msg.wn+1024;
	tow=gal_eph_msg.toe;
	adjustWeekTow(&week,&tow);
	loc_api_eph.q_towMsecOfToe = (uint32_t)(tow*1000.0+0.5);
	loc_api_eph.w_weekOfToe = week;  //  gal time to gps time

	week=gal_eph_msg.wn+1024;
	tow=gal_eph_msg.toc;
	adjustWeekTow(&week,&tow);
	loc_api_eph.q_towMsecOfToc = (uint32_t)(tow*1000.0+0.5);
	loc_api_eph.w_weekOfToc = week;  //  gal time to gps time

	loc_api_InjectApiEphemeris((uint8_t*)(&loc_api_eph), sizeof(loc_api_GnssEphemeris_t));
}
static void decoder_callback_BDSeph_handle(const swift::EphemerisBds &bds_eph_msg)
{
	uint16_t week=0;
	double tow=0.0;
	//fprintf(fp_EPH,"Swift Eph: C%02d iode=%d  %4d %d\n",bds_eph_msg.sat_id,((bds_eph_msg.toe/720)%240), bds_eph_msg.wn,bds_eph_msg.toe);

	loc_api_GnssEphemeris_t loc_api_eph={0};
    SwiftBDSEphToApi(&bds_eph_msg, &loc_api_eph.eph.z_bdsEph);

	loc_api_eph.u_version=VERSION_GNSS_EPHEMERIS;
	loc_api_eph.u_constellation = LOC_API_GNSS_BDS;

	week=bds_eph_msg.wn+1356;
	tow=bds_eph_msg.toe+14;
	adjustWeekTow(&week,&tow);
	loc_api_eph.q_towMsecOfToe = (uint32_t)(tow*1000.0+0.5);
	loc_api_eph.w_weekOfToe = week;  //  bds time to gps time

	week=bds_eph_msg.wn+1356;
	tow=bds_eph_msg.toc+14;
	adjustWeekTow(&week,&tow);
	loc_api_eph.q_towMsecOfToc = (uint32_t)(tow*1000.0+0.5);
	loc_api_eph.w_weekOfToc = week;  //  bds time to gps time

	loc_api_InjectApiEphemeris((uint8_t*)(&loc_api_eph), sizeof(loc_api_GnssEphemeris_t));
}

/*************************************************************************************************************************************/

 // Callback which will be called by the SSRLOS converter when it wants
 // satellite state information at a given time.  The returned array
 // should only include information for the specified satellites which
 // have matching IODEs.
std::size_t fetch_satellite_states(const SatelliteStates* sat_states_in, const uint32_t tow_ms, const uint16_t wn, const std::size_t satellite_iodes_count, const std::array<swift::SatelliteIode, swift::cMaxSats>& satellite_iodes, SatellitePvaArray* sat_states_out)
{
  std::size_t fill_states_count = 0;
  SatellitePvaArray fill_states{};

#if 0
  printf("test time wn=%d tow=%d wn=%d tow=%d\n",wn, tow_ms,sat_states_in->wn,sat_states_in->tow_ms);
  for(uint8_t sys=0;sys<3;sys++)
  {
	  for(uint8_t prn=1;prn<62;prn++)
	  {
		  for (std::size_t iode_idx = 0; iode_idx < satellite_iodes_count && iode_idx < satellite_iodes.size(); iode_idx++)
		  {
		  	   const swift::SatelliteIode requested_iode = satellite_iodes[iode_idx];
		  	   if(prn!=requested_iode.sat_id.number) continue;

		  	   if((0==sys&&swift::GnssId::GPS!=requested_iode.sat_id.constellation)
		  	    ||(1==sys&&swift::GnssId::GAL!=requested_iode.sat_id.constellation)
		  	    ||(2==sys&&swift::GnssId::BDS!=requested_iode.sat_id.constellation))
		  	    {
		  	       continue;
		  	    }
		  	 printf("Iode array sys=%d prn=%02d iode=%d....\n",requested_iode.sat_id.constellation,requested_iode.sat_id.number,requested_iode.iode);

		 }
	  }

  }

  for (std::size_t sat_pva_idx = 0; sat_pva_idx < sat_states_in->sat_states.size(); sat_pva_idx++) {
          const swift::SatellitePva& sat_pva = sat_states_in->sat_states[sat_pva_idx];
          if(sat_pva.sat_id.number==0||(swift::GnssId::BDS==sat_pva.sat_id.constellation&&sat_pva.sat_id.number<19)) continue;
          printf("sat array sys=%d prn=%02d iode=%d....\n",sat_pva.sat_id.constellation,sat_pva.sat_id.number,sat_pva.iode);
        }
#endif

  for (std::size_t sat_pva_idx = 0; sat_pva_idx < sat_states_in->sat_states.size(); sat_pva_idx++)
  {
    const swift::SatellitePva& sat_pva = sat_states_in->sat_states[sat_pva_idx];
    for (std::size_t iode_idx = 0; iode_idx < satellite_iodes_count && iode_idx < satellite_iodes.size(); iode_idx++)
    {
      const swift::SatelliteIode requested_iode = satellite_iodes[iode_idx];
      if (sat_pva.sat_id == requested_iode.sat_id && sat_states_in->tow_ms == tow_ms && sat_states_in->wn == wn)  // do not match iode
      {
        fill_states[fill_states_count] = sat_pva;
        fill_states[fill_states_count].iode = requested_iode.iode;
        fill_states_count++;
        break;
      }
    }
  }
  *sat_states_out = fill_states;

#if 0
    for (std::size_t sat_pva_idx = 0; sat_pva_idx < fill_states_count; sat_pva_idx++) {
            const swift::SatellitePva& sat_pva = fill_states[sat_pva_idx];
            printf("fetch sys=%d prn=%02d iode=%d....\n",sat_pva.sat_id.constellation,sat_pva.sat_id.number,sat_pva.iode);
          }
#endif

  return fill_states_count;
}

static uint8_t satInfoSet(const loc_api_GnssMeasSatBlock_t* pz_GnssSatPosBlk, RoverPosition& rover_position, SatelliteStates& sat_states)
{
  uint8_t u_i = 0;
  uint8_t u_n = 0;
  uint8_t u_gnssid = 0;
  const loc_api_SatPosInfo_t* p_SatPosInfo = NULL;
  // rover position
  rover_position[0] = pz_GnssSatPosBlk->d_xyz[0];
  rover_position[1] = pz_GnssSatPosBlk->d_xyz[1];
  rover_position[2] = pz_GnssSatPosBlk->d_xyz[2];

  // time

  sat_states.tow_ms = pz_GnssSatPosBlk->q_towMsec;
  sat_states.wn = pz_GnssSatPosBlk->w_week;

  for (u_i = 0; u_i < pz_GnssSatPosBlk->u_satCount; u_i++)
  {
    if (u_n >= swift::cMaxSats)
    {
      break;
    }

    if (!pz_GnssSatPosBlk->z_SatPosInfo[u_i].u_valid || (fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[0]) +
      fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[1]) + fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[2])) <= 0.0)
    {
      continue;
    }
    p_SatPosInfo = &pz_GnssSatPosBlk->z_SatPosInfo[u_i];

    // acceleration
    sat_states.sat_states[u_n].acceleration[0] = 0.0;
    sat_states.sat_states[u_n].acceleration[1] = 0.0;
    sat_states.sat_states[u_n].acceleration[1] = 0.0;

    // clock and IODE
    sat_states.sat_states[u_n].clock = p_SatPosInfo->d_satPosClk[3];
    sat_states.sat_states[u_n].clock_rate = p_SatPosInfo->d_satVelClk[3];
    sat_states.sat_states[u_n].iode = p_SatPosInfo->q_iode;

    // position
    sat_states.sat_states[u_n].position[0] = p_SatPosInfo->d_satPosClk[0];
    sat_states.sat_states[u_n].position[1] = p_SatPosInfo->d_satPosClk[1];
    sat_states.sat_states[u_n].position[2] = p_SatPosInfo->d_satPosClk[2];

    // sat_id
    u_gnssid = 0;
    switch (p_SatPosInfo->u_constellation)
    {
    case LOC_API_GNSS_GPS:
      sat_states.sat_states[u_n].sat_id.constellation = swift::GnssId::GPS;break;
    case LOC_API_GNSS_BDS:
      sat_states.sat_states[u_n].sat_id.constellation = swift::GnssId::BDS;break;
    case LOC_API_GNSS_GAL:
      sat_states.sat_states[u_n].sat_id.constellation = swift::GnssId::GAL;break;
    default:
      u_gnssid = 1;
      break;
    }
    if (u_gnssid)
    {
      continue;
    }
    sat_states.sat_states[u_n].sat_id.number = p_SatPosInfo->u_svid;

    // velocity
    sat_states.sat_states[u_n].velocity[0] = p_SatPosInfo->d_satVelClk[0];
    sat_states.sat_states[u_n].velocity[1] = p_SatPosInfo->d_satVelClk[1];
    sat_states.sat_states[u_n].velocity[2] = p_SatPosInfo->d_satVelClk[2];

    sat_states.sat_states[u_n].valid = true;

//    printf("AG-sat sys=%d prn=%02d iode=%d ele=%4.1lf\n",sat_states.sat_states[u_n].sat_id.constellation,sat_states.sat_states[u_n].sat_id.number
//    		,sat_states.sat_states[u_n].iode,p_SatPosInfo->f_elevation*180/PI);
    u_n++;
  }

  if(u_n>0)
  {
	  for (u_i = u_n; u_i < sat_states.sat_states.size(); u_i++)  // remove the unused satellites
	  {
		  memset(&sat_states.sat_states[u_i],0,sizeof(swift::SatellitePva));
	  }
  }

  return u_n;
}
void calculate_elevations(const SatelliteStates& sat_states, const RoverPosition& rover_position, SatelliteElevationArray& elevations) {
  // satellite elevation and azimuth
  for (std::size_t i = 0; i < sat_states.sat_states.size(); i++) {
    if (!sat_states.sat_states[i].valid) {
      continue;
    }

    double elevation_rad = 0.0;
    double azimuth_rad = 0.0;
    swift::internal::wgs_ecef_to_azel(sat_states.sat_states[i].position, rover_position, &azimuth_rad, &elevation_rad);
    elevations[i].sat_id = sat_states.sat_states[i].sat_id;
    elevations[i].elevation_rad = elevation_rad;
  }
}

std::string gnssid_to_string(const swift::GnssId gnss_id) {
  std::string constellation;

  switch (gnss_id) {
  case swift::GnssId::GPS:
    constellation = "GPS";
    break;
  case swift::GnssId::BDS:
    constellation = "BDS";
    break;
  case swift::GnssId::GAL:
    constellation = "GAL";
    break;
  default:
    break;
  }
  return constellation;
}


double get_elevation(const swift::SatelliteDescription& sv_desc, const SatelliteElevationArray& elevations) {
  for (const auto& elevation : elevations) {
    if (elevation.sat_id == sv_desc) {
      return elevation.elevation_rad;
    }
  }
  //assert("Elevation not found for satellite");
  return 0.0;
}

void print_swiftLosJson(const swift::LosCorrections* los_corrections, const SatelliteElevationArray& elevations)
{
#if 0
  static FILE* fp_ssr = NULL;
  if (NULL == fp_ssr)
  {
    fp_ssr = fopen("/home/root/data/los_correction.json", "w");
  }
  if (NULL == fp_ssr)
  {
    return;
  }

  fprintf(fp_ssr,"[\n");
  fprintf(fp_ssr," {\n");
  fprintf(fp_ssr, "  \"flags\": %d,\n", los_corrections->flags);
  fprintf(fp_ssr, "  \"satellites\": [\n");

  for (auto it = los_corrections->sat_correction_block.begin(); it != los_corrections->sat_correction_block.end(); it++)
  {
    auto sat_correction = (*it);

    if (!sat_correction.valid || sat_correction.sv_desc.number == 0)
    {
      //fprintf(fp_ssr, "Invalid sat:%s %d\n", gnssid_to_string(sat_correction.sv_desc.constellation).c_str(),sat_correction.sv_desc.number);
      continue;
    }

    if (it != los_corrections->sat_correction_block.begin()) {
      fprintf(fp_ssr, ",\n");
    }

    fprintf(fp_ssr, "   {\n");

    // clock correction
    fprintf(fp_ssr, "    \"clock\": {\n");
    if (sat_correction.clock.valid) {
      fprintf(fp_ssr, "     \"age\": %d,\n", sat_correction.clock.age_ms);
      fprintf(fp_ssr, "     \"age_of_integrity\": %d,\n", sat_correction.clock.age_of_integrity_ms);
      fprintf(fp_ssr, "     \"bound_mean\": %15lf,\n", sat_correction.clock.bound_mean);
      fprintf(fp_ssr, "     \"bound_std_dev\": %15lf,\n", sat_correction.clock.bound_std_dev);
      fprintf(fp_ssr, "     \"corr\": %15lf,\n", sat_correction.clock.corr);
    }
    fprintf(fp_ssr, "     \"valid\": %d\n", sat_correction.clock.valid);
    fprintf(fp_ssr, "    },\n");

    // code bias corrections
    if (!sat_correction.code_bias.empty()) {
      fprintf(fp_ssr, "    \"code_biases\": [\n");
      for (auto it2 = sat_correction.code_bias.begin(); it2 != sat_correction.code_bias.end(); it2++) {
        auto code = (*it2);

        if (code.signal_code == 255) {
          continue;
        }

        if (it2 != sat_correction.code_bias.begin()) {
          fprintf(fp_ssr, ",\n");
        }

        fprintf(fp_ssr, "     {\n");
        if (code.correction.valid) {
          fprintf(fp_ssr, "      \"age\": %d,\n", code.correction.age_ms);
          fprintf(fp_ssr, "      \"age_of_integrity\": %d,\n", code.correction.age_of_integrity_ms);
          fprintf(fp_ssr, "      \"bias\": %15lf,\n", code.correction.corr);
          fprintf(fp_ssr, "      \"bound_mean\": %15lf,\n", code.correction.bound_mean);
          fprintf(fp_ssr, "      \"bound_std_dev\": %15lf,\n", code.correction.bound_std_dev);
        }
        fprintf(fp_ssr, "      \"signal_code\": %d,\n", (int)code.signal_code);
        fprintf(fp_ssr, "      \"valid\": %d\n", code.correction.valid);
        fprintf(fp_ssr, "     }");
      }
      fprintf(fp_ssr, "\n");
      fprintf(fp_ssr, "    ],\n");
    }

    // elevation
    fprintf(fp_ssr, "    \"elevation_rad\": %15lf,\n", get_elevation(sat_correction.sv_desc, elevations));

    // flags
    fprintf(fp_ssr, "    \"flags\": %d,\n", sat_correction.flags);

    // iono correction
    fprintf(fp_ssr, "    \"iono\": {\n" );
    if (sat_correction.iono.valid) {
      fprintf(fp_ssr, "     \"age\": %d,\n", sat_correction.iono.age_ms);
      fprintf(fp_ssr, "     \"age_of_integrity\": %d,\n", sat_correction.iono.age_of_integrity_ms);
      fprintf(fp_ssr, "     \"bound_mean\": %15lf,\n", sat_correction.iono.bound_mean);
      fprintf(fp_ssr, "     \"bound_std_dev\": %15lf,\n", sat_correction.iono.bound_std_dev);
      fprintf(fp_ssr, "     \"corr\": %15lf,\n", sat_correction.iono.corr);
    }
    fprintf(fp_ssr, "     \"valid\": %d\n", sat_correction.iono.valid);
    fprintf(fp_ssr, "    },\n");

    // orbit correction
    fprintf(fp_ssr, "    \"orbit\": {\n");
    if (sat_correction.orbit.valid) {
      fprintf(fp_ssr, "     \"age\": %d,\n", sat_correction.orbit.age_ms);
      fprintf(fp_ssr, "     \"age_of_integrity\": %d,\n", sat_correction.orbit.age_of_integrity_ms);
      fprintf(fp_ssr, "     \"bound_mean\": %15lf,\n", sat_correction.orbit.bound_mean);
      fprintf(fp_ssr, "     \"bound_std_dev\": %15lf,\n", sat_correction.orbit.bound_std_dev);
      fprintf(fp_ssr, "     \"corr\": %15lf,\n", sat_correction.orbit.corr);
    }
    fprintf(fp_ssr, "     \"valid\": %d\n", sat_correction.orbit.valid);
    fprintf(fp_ssr, "    },\n");

    // phase bias corrections
    if (!sat_correction.phase_bias.empty()) {
      fprintf(fp_ssr, "    \"phase_biases\": [\n");
      for (auto it2 = sat_correction.phase_bias.begin(); it2 != sat_correction.phase_bias.end(); it2++) {
        auto phase = (*it2);

        if (phase.signal_code == 255) {
          continue;
        }

        if (it2 != sat_correction.phase_bias.begin()) {
          fprintf(fp_ssr, ",\n");
        }

        fprintf(fp_ssr, "     {\n");
        if (phase.correction.valid) {
          fprintf(fp_ssr, "      \"age\": %d,\n", phase.correction.age_ms);
          fprintf(fp_ssr, "      \"age_of_integrity\": %d,\n", phase.correction.age_of_integrity_ms);
          fprintf(fp_ssr, "      \"bias\": %15lf,\n", phase.correction.corr);
          fprintf(fp_ssr, "      \"bound_mean\": %15lf,\n", phase.correction.bound_mean);
          fprintf(fp_ssr, "      \"bound_std_dev\": %15lf,\n", phase.correction.bound_std_dev);
        }
        fprintf(fp_ssr, "      \"signal_code\": %d,\n", (int)phase.signal_code);
        fprintf(fp_ssr, "      \"valid\": %d\n", phase.correction.valid);
        fprintf(fp_ssr, "     }");
      }
      fprintf(fp_ssr, "\n");
      fprintf(fp_ssr, "    ],\n");
    }

    // satellite id
    fprintf(fp_ssr, "    \"sat_id\": {\n");
    fprintf(fp_ssr, "     \"constellation\": \"%s\",\n", gnssid_to_string(sat_correction.sv_desc.constellation).c_str());
    fprintf(fp_ssr, "     \"number\": %d\n", (int)sat_correction.sv_desc.number);
    fprintf(fp_ssr, "    },\n");

    // tropo correction
    fprintf(fp_ssr, "    \"tropo\": {\n");
    if (sat_correction.tropo.valid) {
      fprintf(fp_ssr, "     \"age\": %d,\n", sat_correction.tropo.age_ms);
      fprintf(fp_ssr, "     \"age_of_integrity\": %d,\n", sat_correction.tropo.age_of_integrity_ms);
      fprintf(fp_ssr, "     \"bound_mean\": %15lf,\n", sat_correction.tropo.bound_mean);
      fprintf(fp_ssr, "     \"bound_std_dev\": %15lf,\n", sat_correction.tropo.bound_std_dev);
      fprintf(fp_ssr, "     \"corr\": %15lf,\n", sat_correction.tropo.corr);
    }
    fprintf(fp_ssr, "     \"valid\": %d\n", sat_correction.tropo.valid);
    fprintf(fp_ssr, "    },\n");

    // valid
    fprintf(fp_ssr, "    \"valid\": %d\n", sat_correction.valid );

    fprintf(fp_ssr, "   }");
  }
  fprintf(fp_ssr, "\n");
  fprintf(fp_ssr, "  ],\n");

  // SSR provider ID
  fprintf(fp_ssr, "  \"ssr_provider_id\": %d,\n", los_corrections->ssr_provider_id);

  // time
  float tow_s = static_cast<float>(los_corrections->sat_correction_block.at(0).tow_ms) / 1000;

  fprintf(fp_ssr, "  \"time\": {\n");
  fprintf(fp_ssr, "   \"tow\": \"%.1lf\",\n", std::ceil(tow_s * 10.0f) / 10.0f);
  fprintf(fp_ssr, "   \"wn\": %d\n" ,los_corrections->sat_correction_block.at(0).wn);
  fprintf(fp_ssr, "  }\n");

  fprintf(fp_ssr, " }\n");
  fprintf(fp_ssr, "]\n");

#endif

}
void ag_print_output(const loc_api_GnssMeasSatBlock_t* pz_GnssSatPosBlk, const swift::LosCorrections& los_corrections
		, const SatelliteElevationArray& elevations)
{

#if 0

  static FILE* fp_ssr = NULL;
  if (NULL == fp_ssr)
  {
    fp_ssr = fopen("/home/root/data/agsat-corr.log", "w");
  }
  if (NULL == fp_ssr)
  {
    return;
  }

  fprintf(fp_ssr,"Time of Asensing: %04d %8.1lf****************************\n",pz_GnssSatPosBlk->w_week,pz_GnssSatPosBlk->q_towMsec*0.001);
  // SSR provider ID
  fprintf(fp_ssr, "ssr_provider_id: %d", los_corrections.ssr_provider_id);
  // time
  float tow_s = static_cast<float>(los_corrections.sat_correction_block.at(0).tow_ms) / 1000;
  fprintf(fp_ssr, " time: %d %8.1lf\n", los_corrections.sat_correction_block.at(0).wn, std::ceil(tow_s * 10.0f) / 10.0f);

  uint8_t u_nag = 0;
  uint8_t u_nswift = 0;
  char c_sat[4] = "";
  for (uint8_t u_i = 0; u_i < pz_GnssSatPosBlk->u_satCount; u_i++)
  {
      if (!pz_GnssSatPosBlk->z_SatPosInfo[u_i].u_valid || (fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[0]) +
        fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[1]) + fabs(pz_GnssSatPosBlk->z_SatPosInfo[u_i].d_satPosClk[2])) <= 0.0)
      {
        continue;
      }
      const loc_api_SatPosInfo_t* p_SatPosInfo = &pz_GnssSatPosBlk->z_SatPosInfo[u_i];

      switch (p_SatPosInfo->u_constellation)
      {
      case LOC_API_GNSS_GPS:
        sprintf(c_sat, "G%02d", p_SatPosInfo->u_svid); break;
      case LOC_API_GNSS_BDS:
        sprintf(c_sat, "C%02d", p_SatPosInfo->u_svid); break;
      case LOC_API_GNSS_GAL:
        sprintf(c_sat, "E%02d", p_SatPosInfo->u_svid); break;
      default:
        continue;
        break;
      }
      fprintf(fp_ssr, "%s iode=%4d ele=%7.1lf\n", c_sat, p_SatPosInfo->q_iode, p_SatPosInfo->f_elevation * 180 / PI);
      u_nag++;
  }

  fprintf(fp_ssr, "Swift correction:\n");
  for (auto it = los_corrections.sat_correction_block.begin(); it != los_corrections.sat_correction_block.end(); it++)
  {
    auto sat_correction = (*it);
    switch (sat_correction.sv_desc.constellation)
    {
    case swift::GnssId::GPS :
      sprintf(c_sat, "G%02d", sat_correction.sv_desc.number); break;
    case swift::GnssId::BDS:
      sprintf(c_sat, "C%02d", sat_correction.sv_desc.number); break;
    case swift::GnssId::GAL:
      sprintf(c_sat, "E%02d", sat_correction.sv_desc.number); break;
    default:
      continue;
      break;
    }
    int iode = 0;
    for (uint8_t u_i = 0; u_i < 10; u_i++)
    {
      if (sat_correction.flags & (1 << u_i))
      {
        iode += 1 << u_i;
      }
    }
    fprintf(fp_ssr, "%s iode=%4d valid=%d flag=%d\n", c_sat, iode, sat_correction.valid, sat_correction.flags);
    if (!sat_correction.valid || sat_correction.sv_desc.number == 0)
    {
      continue;
    }
    u_nswift++;
    // orbit correction
    fprintf(fp_ssr, "orbit:");
    if (sat_correction.orbit.valid) {
      fprintf(fp_ssr, " age: %7.1lf", sat_correction.orbit.age_ms * 0.001);
      fprintf(fp_ssr, " age_of_integrity: %7.1lf", sat_correction.orbit.age_of_integrity_ms * 0.001);
      fprintf(fp_ssr, " bound_mean: %7.1lf", sat_correction.orbit.bound_mean);
      fprintf(fp_ssr, " bound_std_dev: %7.1lf", sat_correction.orbit.bound_std_dev);
      fprintf(fp_ssr, " corr: %7.3lf", sat_correction.orbit.corr);
    }
    fprintf(fp_ssr, "  valid: %d\n", sat_correction.orbit.valid);

    // clock correction
    fprintf(fp_ssr, "clock:");
    if (sat_correction.clock.valid) {
      fprintf(fp_ssr, " age: %7.1lf", sat_correction.clock.age_ms*0.001);
      fprintf(fp_ssr, " age_of_integrity: %7.1lf", sat_correction.clock.age_of_integrity_ms*0.001);
      fprintf(fp_ssr, " bound_mean: %7.1lf", sat_correction.clock.bound_mean);
      fprintf(fp_ssr, " bound_std_dev: %7.1lf", sat_correction.clock.bound_std_dev);
      fprintf(fp_ssr, " corr: %7.3lf", sat_correction.clock.corr);
    }
    fprintf(fp_ssr, "  valid: %d\n", sat_correction.clock.valid);

    // code bias corrections
    if (!sat_correction.code_bias.empty()) {
      fprintf(fp_ssr, "code :");
      for (auto it2 = sat_correction.code_bias.begin(); it2 != sat_correction.code_bias.end(); it2++) {
        auto code = (*it2);

        if (code.signal_code == 255) {
          continue;
        }

        if (it2 != sat_correction.code_bias.begin()) {
          fprintf(fp_ssr, ",     ");
        }

        if (code.correction.valid) {
          fprintf(fp_ssr, " age: %7.1lf", code.correction.age_ms * 0.001);
          fprintf(fp_ssr, " age_of_integrity: %7.1lf", code.correction.age_of_integrity_ms * 0.001);
          fprintf(fp_ssr, " bound_mean: %7.1lf", code.correction.bound_mean);
          fprintf(fp_ssr, " bound_std_dev: %7.1lf", code.correction.bound_std_dev);
          fprintf(fp_ssr, " corr: %7.1lf", code.correction.corr);
        }
        fprintf(fp_ssr, " signal_code: %3d,", (int)code.signal_code);
        fprintf(fp_ssr, " valid: %d\n", code.correction.valid);
      }
    }

    // phase bias corrections
    if (!sat_correction.phase_bias.empty()) {
      fprintf(fp_ssr, "phase:");
      for (auto it2 = sat_correction.phase_bias.begin(); it2 != sat_correction.phase_bias.end(); it2++) {
        auto phase = (*it2);

        if (phase.signal_code == 255) {
          continue;
        }

        if (it2 != sat_correction.phase_bias.begin()) {
          fprintf(fp_ssr, ",     ");
        }

        if (phase.correction.valid) {
          fprintf(fp_ssr, " age: %7.1lf", phase.correction.age_ms * 0.001);
          fprintf(fp_ssr, " age_of_integrity: %7.1lf", phase.correction.age_of_integrity_ms * 0.001);
          fprintf(fp_ssr, " bound_mean: %7.1lf", phase.correction.bound_mean);
          fprintf(fp_ssr, " bound_std_dev: %7.1lf", phase.correction.bound_std_dev);
          fprintf(fp_ssr, " corr: %7.1lf", phase.correction.corr);
        }
        fprintf(fp_ssr, " signal_code: %3d,", (int)phase.signal_code);
        fprintf(fp_ssr, " valid: %d\n", phase.correction.valid);
      }
    }

    // iono correction
    fprintf(fp_ssr, "iono :" );
    if (sat_correction.iono.valid) {
      fprintf(fp_ssr, " age: %7.1lf", sat_correction.iono.age_ms * 0.001);
      fprintf(fp_ssr, " age_of_integrity: %7.1lf", sat_correction.iono.age_of_integrity_ms * 0.001);
      fprintf(fp_ssr, " bound_mean: %7.1lf", sat_correction.iono.bound_mean);
      fprintf(fp_ssr, " bound_std_dev: %7.1lf", sat_correction.iono.bound_std_dev);
      fprintf(fp_ssr, " corr: %7.1lf", sat_correction.iono.corr);
    }
    fprintf(fp_ssr, " valid: %d\n", sat_correction.iono.valid);

    // tropo correction
    fprintf(fp_ssr, "tropo:");
    if (sat_correction.tropo.valid) {
      fprintf(fp_ssr, " age: %7.1lf", sat_correction.tropo.age_ms * 0.001);
      fprintf(fp_ssr, " age_of_integrity: %7.1lf", sat_correction.tropo.age_of_integrity_ms * 0.001);
      fprintf(fp_ssr, " bound_mean: %7.1lf", sat_correction.tropo.bound_mean);
      fprintf(fp_ssr, " bound_std_dev: %7.1lf", sat_correction.tropo.bound_std_dev);
      fprintf(fp_ssr, " corr: %7.1lf", sat_correction.tropo.corr);
    }
    fprintf(fp_ssr, " valid: %d\n", sat_correction.tropo.valid);
    fprintf(fp_ssr, "\n");
  }
  printf("ns ag=%d  ns swift=%d\n",u_nag,u_nswift);

#endif

}
void print_satinfo(RoverPosition& rover_position, SatelliteStates& sat_states)
{

#if 0
  static FILE* fp_sat = NULL;
  if (NULL == fp_sat)
  {
    fp_sat = fopen("/home/root/data/satinfo.json", "w");
    if (NULL != fp_sat)fprintf(fp_sat, "[\n");
  }
  if (NULL == fp_sat)
  {
    return;
  }

  fprintf(fp_sat, "  {\n");
  fprintf(fp_sat, "    \"rover_position\": [\n");
  fprintf(fp_sat, "      %lf,\n", rover_position[0]);
  fprintf(fp_sat, "      %lf,\n", rover_position[1]);
  fprintf(fp_sat, "      %lf\n", rover_position[2]);
  fprintf(fp_sat, "    ],\n");

  fprintf(fp_sat, "    \"sat_states\": [\n");
  for (auto it = sat_states.sat_states.begin(); it != sat_states.sat_states.end(); it++)
  {
    if (it->sat_id.number == 0)
    {
      continue;
    }

    if (it != sat_states.sat_states.begin()) {
      fprintf(fp_sat, ",\n");
    }

    fprintf(fp_sat, "      {\n");
    fprintf(fp_sat, "        \"acceleration\": [\n");
    fprintf(fp_sat, "          %lf,\n", it->acceleration[0]);
    fprintf(fp_sat, "          %lf,\n", it->acceleration[1]);
    fprintf(fp_sat, "          %lf\n", it->acceleration[2]);
    fprintf(fp_sat, "        ],\n");

    fprintf(fp_sat, "        \"clock\": %lf,\n", it->clock);
    fprintf(fp_sat, "        \"clock_rate\":  %lf,\n", it->clock_rate);
    fprintf(fp_sat, "        \"iode\": %d,\n", it->iode);

    fprintf(fp_sat, "        \"position\": [\n");
    fprintf(fp_sat, "          %lf,\n", it->position[0]);
    fprintf(fp_sat, "          %lf,\n", it->position[1]);
    fprintf(fp_sat, "          %lf\n", it->position[2]);
    fprintf(fp_sat, "        ],\n");

    fprintf(fp_sat, "        \"sat_id\": {\n");
    fprintf(fp_sat, "          \"constellation\": \"%s\",\n", gnssid_to_string(it->sat_id.constellation).c_str());
    fprintf(fp_sat, "          \"number\": %d\n", (int)it->sat_id.number);
    fprintf(fp_sat, "        },\n");

    fprintf(fp_sat, "        \"velocity\": [\n");
    fprintf(fp_sat, "          %lf,\n", it->velocity[0]);
    fprintf(fp_sat, "          %lf,\n", it->velocity[1]);
    fprintf(fp_sat, "          %lf\n", it->velocity[2]);
    fprintf(fp_sat, "        ]\n");

    fprintf(fp_sat, "      }");
  }
  fprintf(fp_sat, "\n");
  fprintf(fp_sat, "    ], \n");

  fprintf(fp_sat, "    \"time\": {\n");
  fprintf(fp_sat, "      \"tow\": %d,\n", (int)(sat_states.tow_ms*0.001+0.5));
  fprintf(fp_sat, "      \"wn\": %d\n", sat_states.wn);
  fprintf(fp_sat, "    }\n");

  fprintf(fp_sat, "  },\n");
  //fprintf(fp_sat, "]\n");

#endif

}
static void init_integrity_config(swift::IntegrityConfig* config) {
  // disable flags checking
  config->enable_flags_checking = false;
  config->enable_bounds_calculation = true;
  config->enable_protection_level_vs_threshold_comparison=false;
  config->enable_local_tropo_integrity_check= true;

  // bounds configuration values
  config->integrity_risk = 1e-5;
  config->clock_integrity_threshold_m = 10000.0;
  config->orbit_integrity_threshold_m = 10000.0;
  config->code_bias_integrity_threshold_m = 10000.0;
  config->phase_bias_integrity_threshold_m = 100000.0;
  config->iono_integrity_threshold_m = 10000.0;
  config->tropo_integrity_threshold_m = 10000.0;
}

// put ssr stream of swift
static int swift_adapter_push_ssr(const uint8_t* ssr_buff, uint32_t length)
{
  // push correction data stream
  auto ret_handle_client = swift_client.handle_rtcm_message(ssr_buff, length);
  (void)ret_handle_client;

  return 0;

}

static uint32_t rtcm_crc24q(const uint8_t *buff, int len)
{
    uint32_t crc=0;
    int i;
    static const uint32_t tbl_CRC24Q[]={
        0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
        0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
        0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
        0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
        0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
        0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
        0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
        0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
        0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
        0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
        0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
        0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
        0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
        0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
        0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
        0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
        0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
        0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
        0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
        0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
        0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
        0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
        0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
        0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
        0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
        0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
        0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
        0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
        0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
        0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
        0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
        0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
    };

    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

static int process_rtcm_stream(const uint8_t* ssr_buff, uint32_t length)
{
  uint8_t data = 0;
  uint32_t w_i = 0;
  uint8_t status=0;

  for (w_i = 0; w_i < length; w_i++)
  {
    data = ssr_buff[w_i];
    // read preamble and check that it in fact is the preamble
    if (gz_len_ssr_buff == 0)
    {
      if (RTCM3PREAMB != swift::rtcm_decoder::getbitu8(&data, 0, 8))
      {
        continue;
      }
      pu_ssr_buff[gz_len_ssr_buff++] = data;
      continue;
    }
    if (gz_len_ssr_buff == 1)
    {
      if (0 != swift::rtcm_decoder::getbitu8(&data, 0, 6))
      {
    	printf("Wrong RTCM3PREAMB\n");
        gz_len_ssr_buff = 0;
        continue;
      }
    }
    pu_ssr_buff[gz_len_ssr_buff++] = data;
    if (gz_len_ssr_buff < 3)
    {
      continue;
    }

    // sum 6 because the length is represented by the last 10 bits of the 3 bytes
    if (gz_len_ssr_buff == 3)
    {
      payload_length = swift::rtcm_decoder::getbitu16(pu_ssr_buff, 14, 10);
      if(payload_length<=0)
      {
        payload_i = 0;
        payload_length=0;
        gz_len_ssr_buff = 0;
      }
      continue;
    }


    // read payload message into payload
    if (payload_i < payload_length)
    {
      payload[payload_i++] = data;
    }
    // read CRC for the next message to not include it
    if (gz_len_ssr_buff < (payload_length) + 6)
    {
      continue;
    }

    /* check parity */
    if (rtcm_crc24q(pu_ssr_buff, payload_length+3) != swift::rtcm_decoder::getbitu32(pu_ssr_buff, (payload_length+3) * 8, 24))
    {
      printf("rtcm3 crc24q check error: len=%d\n", payload_length+3);
      payload_i = 0;
      payload_length=0;
      gz_len_ssr_buff = 0;
      continue;
    }

    /* decode rtcm3 message */
    if (payload_i > 0 && payload_i == payload_length)
    {
      status+=swift_adapter_push_ssr(payload, payload_length);
    }

    payload_i = 0;
    payload_length=0;
    gz_len_ssr_buff = 0;
  }

  return status;
}

static void *SwiftSdk_threadFunc(void *arg)
{
	uint8_t buff[1024*50]="";
	uint32_t len=0;
	//struct  timeval tv;

	while(1)
	{
		if(gz_rt_len_ssr_buff>0)
		{
			std::lock_guard<std::mutex> locker(Mutex_SwiftSDK);
			memcpy(buff,RT_ssr_stream_buff,gz_rt_len_ssr_buff);
			len=gz_rt_len_ssr_buff;
			gz_rt_len_ssr_buff=0;
		}

		if(len>0)
		{
#if 0
			static FILE* fp_rtcm=NULL;
			if(NULL==fp_rtcm)
			{
			  fp_rtcm = fopen("/home/root/data/corrections.rtcm","wb+");
			}
			uint32_t pos=0;
			if(NULL!=fp_rtcm)
			{
			  if(strstr((char*)buff,"ICY 200 OK")&&len>pos)
			  {
			    pos=14;
			  }
			  fwrite(buff+pos,len-pos,1u,fp_rtcm);
		    }
#endif
			process_rtcm_stream(buff,len);
		}

        /*******  note: it is need to sleep 500 ms  *********/
        
		//tv.tv_sec = 0;
		//tv.tv_usec = 500*1000u;
		//select(0,NULL,NULL,NULL,&tv);
	}
}

void SwiftSdk_StartUp(void)
{
	//pthread_t ptid;
	//const char *paramlist = "SwiftSdk_StartUp";
	//pthread_create(&ptid,NULL,SwiftSdk_threadFunc,(void *)paramlist);
	//pthread_setname_np(ptid,paramlist);

}

/**
 * @brief Ntrip client recv callback
 * @param[in]
 * @return      None
 */
void ntrip_client_callback_receive_data_handler(uint8_t* buf, uint32_t len)
{
  if(len>0)
  {
	 //printf("ntrip stream received,len=%d\n",len);
	 if((gz_rt_len_ssr_buff+len)<1024*50)
	 {
		  std::lock_guard<std::mutex> locker(Mutex_SwiftSDK);
		  memcpy(RT_ssr_stream_buff+gz_rt_len_ssr_buff,buf,len);
		  gz_rt_len_ssr_buff+=len;
	 }
	 else
	 {
		 assert(false&& "gz_rt_len_ssr_buff is outlier");
	 }
	 ntrip_SSRBuffSum_len += len;
	 if(ntrip_SSRBuffSum_len>1e+9) ntrip_SSRBuffSum_len=0;

  }
  return;
}
uint64_t SwiftSdk_GetSSRSum_len(void)
{
	return ntrip_SSRBuffSum_len;

}

// initialize adapter
void swift_adapter_init()
{
  // configuration structure
  init_integrity_config(&swift_config.integrity);  // just initialize once

  // prepare callback structure
  swift_callbacks.decoder_callbacks.gps_eph_msg_callback=std::bind(decoder_callback_GPSeph_handle, std::placeholders::_1);
  swift_callbacks.decoder_callbacks.gal_eph_msg_callback=std::bind(decoder_callback_GALeph_handle, std::placeholders::_1);
  swift_callbacks.decoder_callbacks.bds_eph_msg_callback=std::bind(decoder_callback_BDSeph_handle, std::placeholders::_1);

  swift_callbacks.correction_callbacks.satellite_states_callback = std::bind(fetch_satellite_states, &swift_sat_states, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);

  // create SsrClient object
  // init client
  auto ret_init_client = swift_client.init(swift_config, swift_callbacks); // just initialize once
  assert((ret_init_client == swift::ReturnCode::SUCCESS) && "Failed to init SSR2LOS client");
  printf("swift_client init code=%u\n",(uint8_t)ret_init_client);
  (void)ret_init_client;
}

/**
  * @brief sdk running of swift
  * @param[in] pz_GnssSatPosBlk - information of satllites
  * @return    none
  */
void swift_sdk_runing(const loc_api_GnssMeasSatBlock_t* pz_GnssSatPosBlk)
{
  if (NULL == pz_GnssSatPosBlk)
  {
    return;
  }
  // rover position
  RoverPosition rover_position;
  swift::LosCorrections* los_corrections = (swift::LosCorrections*)malloc(sizeof(swift::LosCorrections));
  memset(los_corrections, 0, sizeof(swift::LosCorrections));

  if (pz_GnssSatPosBlk->w_week > 0 && (fabs(pz_GnssSatPosBlk->d_xyz[0]) + fabs(pz_GnssSatPosBlk->d_xyz[1]) + fabs(pz_GnssSatPosBlk->d_xyz[2])) > 0)
  {
    std::lock_guard<std::mutex> locker(Mutex_SwiftSDK);

    // time and satellite states
    if (!satInfoSet(pz_GnssSatPosBlk, rover_position, swift_sat_states))
    {
      return;
    }
    // calculate ssr2los corrections
    auto ret = swift_client.calculate_corrections(rover_position, swift_sat_states.tow_ms, swift_sat_states.wn, los_corrections);
    printf("swift_client calculate_corrections code=%u\n", (uint8_t)ret);
    if (ret != swift::ReturnCode::SUCCESS)
    {
      (void)ret;
      free(los_corrections);
      return;
    }
    (void)ret;
  }
  else
  {
    free(los_corrections);
    return;
  }

  //print_satinfo(rover_position, swift_sat_states);

  // compute elevations to include in output
  SatelliteElevationArray sat_elevations;
  //calculate_elevations(swift_sat_states, rover_position, sat_elevations);

  // print the computed corrections in a JSON-like format
  print_swiftLosJson(los_corrections, sat_elevations);

  // convert ssr struct of swift to asensing
  loc_api_ssrLosBlock_t* pz_SsrLosBlk = (loc_api_ssrLosBlock_t*)malloc(sizeof(loc_api_ssrLosBlock_t));
  memset(pz_SsrLosBlk, 0, sizeof(loc_api_ssrLosBlock_t));
  ssr_cvt_SsrLosBlk_Swift2Ag(pz_GnssSatPosBlk->w_week, (uint32_t)round(pz_GnssSatPosBlk->q_towMsec * 0.001), rover_position, los_corrections, pz_SsrLosBlk);

  loc_api_InjectSsrLosBlock(pz_SsrLosBlk);

  free(los_corrections);
  free(pz_SsrLosBlk);
  return;

}


#endif // APP_SWIFT_SSR
