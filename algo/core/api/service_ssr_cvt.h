/**@file        service_ssr_cvt.h
 * @brief       ssr data convert functions
 *              1. QianXun SSR data struct to Asensing SSR struct
 *              2. GeeSpace SSR data struct to Asensing SSR struct
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/29  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SERVICE_SSR_CVT_H__
#define __SERVICE_SSR_CVT_H__

#include "gnss_type.h"
#include "qxsi_ssr2los_type_def.h"
#include "dpi_sdk.h"

BEGIN_DECL

/**
 * @brief Convert SSR los from qxsi to asensing
 * @param[in] pz_qxsi_los - Qxsi SSR los data
 * @param[in] pz_ag_los - Asensing SSR los data
 * @return    satellite number
 */
uint8_t ssr_cvt_SsrLosBlk_Qxwz2Ag(const qxsi_station_los_t* pz_qxsi_los,
  gnss_ssrLosBlock_t* pz_ag_los);

/**
 * @brief Convert SSR los from dayou to asensing
 * @param[in] pz_dysk_los - Dayou SSR los data
 * @param[in] pz_ag_los - Asensing SSR los data
 * @return    Satellite Number
 */
uint8_t ssr_cvt_SsrLosBlk_Dysk2Ag(const DySatLos_t* pz_dysk_los,
  gnss_ssrLosBlock_t* pz_ag_los);

/**
 * @brief Convert GNSS measurement block from asensing to qxsi
 * @param[in]  pz_site_coor -approximate coordinate of site
 * @param[in]  pz_ag_meas   - asensing GNSS measurement block
 * @param[out] pz_qxsi_station_info - qxsi station information
 * @return     0 represent failure, 1 represent success
 */
uint8_t cvt_GnssMeasBlk_Ag2Qxwz(const gnss_PositionFix_t* pz_posFix,
  const GnssMeasBlock_t* pz_ag_meas_block,
  qxsi_station_info_t* pz_qxsi_station_info);

/**
 * @brief Convert GNSS measurement block from asensing to qxsi
 * @param[in]  pz_site_coor -approximate coordinate of site
 * @param[in]  pz_SatSigMeasCollect   - asensing GNSS measurement block
 * @param[out] pz_qxsi_station_info - qxsi station information
 * @return     0 represent failure, 1 represent success
 */
uint8_t cvt_SatSigMeasCollect_Ag2Qxwz(const gnss_PositionFix_t* pz_posFix,
  const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect,
  qxsi_station_info_t* pz_qxsi_station_info);

END_DECL

#endif
