/**@file        loc_core_nmea_api.c
 * @brief       Location core engine API for NMEA generation
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/11  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_def.h"
#include "gnss_common.h"
#include "sm_nmea.h"
#include "loc_core_nmea_api.h"

static gnss_PositionFix_t z_PositionFixCached = { 0 };

/**
 * @brief Compare the input position fix are same or not
 * @param[in] pz_LocPositionFix : Consoilddate position fix
 * @param[in] pz_GnssPositionFix: Gnss position fix
 * @return TRUE  - success
 *         FALSE - fail
 */
static BOOL loc_api_nmea_compare_position_fix(const loc_api_ConsoildatedPositionFix_t* pz_LocPositionFix,
  gnss_PositionFix_t* pz_GnssPositionFix)
{
  if (pz_LocPositionFix->q_towMsec != pz_GnssPositionFix->z_gpsTime.q_towMsec)
  {
    return FALSE;
  }

  if (pz_LocPositionFix->w_week != pz_GnssPositionFix->z_gpsTime.w_week)
  {
    return FALSE;
  }

  if (pz_LocPositionFix->u_fixFlag != pz_GnssPositionFix->u_fixFlag)
  {
    return FALSE;
  }

  if (pz_LocPositionFix->u_fixSource != pz_GnssPositionFix->u_fixSource)
  {
    return FALSE;
  }

  if (!DBL_EQU(pz_LocPositionFix->d_lla[0], pz_GnssPositionFix->d_lla[0]))
  {
    return FALSE;
  }

  if (!DBL_EQU(pz_LocPositionFix->d_lla[1], pz_GnssPositionFix->d_lla[1]))
  {
    return FALSE;
  }

  if (!DBL_EQU(pz_LocPositionFix->d_lla[2], pz_GnssPositionFix->d_lla[2]))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief create gga string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gga string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_gga(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* gga)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_gga(&z_PositionFixCached, gga);
}

/**
 * @brief create rmc string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] rmc string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_rmc(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* rmc)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_rmc(&z_PositionFixCached, rmc);
}

/**
 * @brief create gsa string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gsa string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t loc_api_nmea_create_gsa(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix,
  uint8_t gsa[][LOC_API_NMEA_STRING_LEN], uint8_t gsa_max)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_gsa(&z_PositionFixCached, gsa, gsa_max);
}

/**
 * @brief create gsv string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gsv string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t loc_api_nmea_create_gsv(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix,
  uint8_t gsv[][LOC_API_NMEA_STRING_LEN], uint8_t gsv_max)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_gsv(&z_PositionFixCached, gsv, gsv_max);
}

/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] vtg string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_vtg(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* vtg)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_vtg(&z_PositionFixCached, vtg);
}

/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] zda string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_zda(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* zda)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_zda(&z_PositionFixCached, zda);
}

/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] ntr string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_ntr(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* ntr)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_create_ntr(&z_PositionFixCached, ntr);
}

/**
 * @brief create hdt string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] hdt string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_hdt(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_hdt)
{
  gnss_OrientFix_t z_orientFix = { 0 };
  gnss_cvt_LocApiOrientFix_To_GnssOrientFix(pz_ortFix, &z_orientFix);
  return sm_nmea_creat_hdt(&z_orientFix, pu_hdt);
}

/**
 * @brief create tra string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] tra string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_tra(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_tra)
{
  gnss_OrientFix_t z_orientFix = { 0 };
  gnss_cvt_LocApiOrientFix_To_GnssOrientFix(pz_ortFix, &z_orientFix);
  return sm_nmea_creat_tra(&z_orientFix, pu_tra);
}

/**
 * @brief create ksxt string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] tra string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_ksxt(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_ksxt)
{
  gnss_OrientFix_t z_orientFix = { 0 };
  gnss_cvt_LocApiOrientFix_To_GnssOrientFix(pz_ortFix, &z_orientFix);
  return sm_nmea_creat_ksxt(&z_orientFix, pu_ksxt);
}

/**
 * @brief create zone string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] zone string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_zone(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* zone)
{
  if (FALSE == loc_api_nmea_compare_position_fix(pz_PositionFix, &z_PositionFixCached))
  {
    gnss_cvt_LocApiPositionFix_To_GnssPositionFix(pz_PositionFix, &z_PositionFixCached);
  }

  return sm_nmea_creat_zone(&z_PositionFixCached, zone);
}