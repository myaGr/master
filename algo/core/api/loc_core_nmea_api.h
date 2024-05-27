/**@file        loc_core_nmea_api.h
 * @brief       Location core engine interface for NMEA generation
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

#ifndef __LOC_CORE_NMEA_API_H__
#define __LOC_CORE_NMEA_API_H__

#include "loc_core_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOC_API_NMEA_STRING_LEN     (128)
#define LOC_API_NMEA_GSV_STRING_LEN (128)
#define LOC_API_NMEA_GSA_STRING_LEN (128)
#define LOC_API_NMEA_GSA_NUM        (3)
#define LOC_API_NMEA_GSV_NUM        (27)

/**
 * @brief create gga string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gga string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_gga(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* gga);

/**
 * @brief create rmc string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] rmc string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_rmc(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* rmc);

/**
 * @brief create gsa string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gsa string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t loc_api_nmea_create_gsa(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix,
  uint8_t gsa[][LOC_API_NMEA_STRING_LEN], uint8_t gsa_max);

/**
 * @brief create gsv string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] gsv string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t loc_api_nmea_create_gsv(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix,
  uint8_t gsv[][LOC_API_NMEA_STRING_LEN], uint8_t gsv_max);


/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] vtg string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_vtg(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* vtg);


/**
 * @brief create zda string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] zda string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_zda(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* zda);


/**
 * @brief create ntr string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] ntr string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_ntr(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* ntr);

/**
 * @brief create hdt string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] hdt string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_hdt(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_hdt);

/**
 * @brief create tra string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] tra string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_tra(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_tra);

/**
 * @brief create ksxt string from input orient fix
 * @param[in] pz_ortFix : orient result
 * @param[out] tra string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_ksxt(const loc_api_orient_report_t* pz_ortFix, uint8_t* pu_ksxt);

/**
 * @brief create zone string from input position fix
 * @param[in] pz_PositionFix : consoilddate position fix
 * @param[out] zone string
 * @return 1 - success
 *         0 - fail
 */
uint8_t loc_api_nmea_create_zone(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* zone);

#ifdef __cplusplus
}
#endif

#endif