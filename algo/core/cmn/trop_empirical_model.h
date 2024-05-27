/**@file        trop_empirical_model.h
 * @brief       the empirical model for troposphere
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/29  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __TROP_EMPIRICAL_MODEL_H__
#define __TROP_EMPIRICAL_MODEL_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL
/**
 * @brief using Saastamoinen(SAAS) model to calculate the empirical value of troposphere
 * @param[in]  z_obsTime observation time of calculating the empirical value of troposphere
 * @param[in]  pd_LLA geodetic coordinates of site, rad
 * @param[in]  d_humi the humidity of site,usually set to 0.7
 * @param[in]  u_metOpt whether using global pressure temperature(GPT) model,1 represent using GPT
 * @param[out] pd_zenithDry is the zenith tropospheric delaly of dry part
 * @param[out] pd_zenithWet is the zenith tropospheric delaly of wet part
 * @return 1 represent failure, 0 reprenset success
 */
uint8_t usingSaasModelCalZenithTropDelay(GpsTime_t z_obsTime, const double pd_LLA[3], double d_humi, uint8_t u_metOpt, double* pd_zenithDry, double* pd_zenithWet);

/**
 * @brief using GMF model to calculate the map value of troposphere
 * @param[in]  d_doy is the day of year
 * @param[in]  pd_LLA geodetic coordinates of site, rad
 * @param[in]  d_ele the elevation of satellite, rad
 * @param[out] pd_mapDry is the tropospheric map of dry part
 * @param[out] pd_mapWet is the tropospheric map of wet part
 * @return     uint8_t  1 represent failure, 0 reprenset success
 */
uint8_t usingGMFcalculateTropMap(double d_doy, const double pd_LLA[3], double d_ele, double* pd_mapDry, double* pd_mapWet);
END_DECL
#endif