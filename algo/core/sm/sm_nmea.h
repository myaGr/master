/**@file        sm_nmea.h
 * @brief
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/24  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SM_NMEA_H__
#define __SM_NMEA_H__

#include "cmn_def.h"
#include "gnss_type.h"

#include "loc_core_nmea_api.h"

BEGIN_DECL

#define NMEA_STRING_LEN     (LOC_API_NMEA_STRING_LEN)
#define NMEA_GSV_STRING_LEN (LOC_API_NMEA_GSV_STRING_LEN)
#define NMEA_GSA_STRING_LEN (LOC_API_NMEA_GSA_STRING_LEN)
#define NMEA_GSA_NUM        (LOC_API_NMEA_GSA_NUM)
#define NMEA_GSV_NUM        (LOC_API_NMEA_GSV_NUM)

/**
 * @brief create gga string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gga string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gga(const gnss_PositionFix_t* pz_PositionFix, uint8_t* gga);

/**
 * @brief create rmc string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] rmc string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_rmc(const gnss_PositionFix_t* pz_PositionFix, uint8_t* rmc);

/**
 * @brief create gsa string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gsa string
 * @return TRUE - success
 *         FALSE - fail
 */
int32_t sm_nmea_create_gsa(const gnss_PositionFix_t* pz_PositionFix, uint8_t gsa[][NMEA_GSA_STRING_LEN], uint8_t u_gsa_max);

/**
 * @brief create gsv string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gsv string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t sm_nmea_create_gsv(const gnss_PositionFix_t* pz_PositionFix, uint8_t gsv[][NMEA_GSV_STRING_LEN], uint8_t u_gsv_max);

/**
 * @brief create gga string from input position fix
 * @param[in] pz_location_report
 * @param[out] gga string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gga_deprecate(const loc_api_location_report_t* pz_location_report, uint8_t* gga);


/**
 * @brief create gsa string from input position fix
 * @param[in] pz_location_report
 * @param[out] gsa string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gsv_deprecate(const loc_api_location_report_t* pz_location_report, uint8_t gsv[][NMEA_GSV_STRING_LEN], uint8_t* pu_gsv_num, char gsv_max);

/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] vtg string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_vtg(const gnss_PositionFix_t* pz_PositionFix, uint8_t* vtg);

/**
 * @brief create zda string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] zda string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_zda(const gnss_PositionFix_t* pz_PositionFix, uint8_t* zda);

/**
 * @brief create ntr string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] ntr string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_ntr(const gnss_PositionFix_t* pz_PositionFix, uint8_t* ntr);

/**
 * @brief create zone string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] zone string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_zone(const gnss_PositionFix_t* pz_PositionFix, uint8_t* zone);

/**
 * @brief create hdt string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] hdt string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_hdt(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_hdt);

/**
 * @brief create tra string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] tra string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_tra(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_tra);

/**
 * @brief create ksxt string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] ksxt string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_ksxt(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_ksxt);


#endif

END_DECL

