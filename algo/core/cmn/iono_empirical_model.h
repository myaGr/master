/**@file        iono_empirical_model.h
 * @brief       Ionosphere Empirical Models
 * @details     Broadcast ionosphere model (or called Klobuchar model)
 * @author      houxiaowei
 * @date        2022/8/29
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/8/29   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef LOCATION_ENGINE_IONO_EMPIRICAL_MODEL_H
#define LOCATION_ENGINE_IONO_EMPIRICAL_MODEL_H
#include "gnss_type.h"

/**
 * @brief ionosphere mapping function; compute ionospheric delay mapping function by single layer model
 * @param[in] pos receiver position {lat,lon,h} (rad,m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @return ionospheric mapping function
 */
extern double ion_Mapfution(const double *pos, const double *azel);

/**
 * @brief compute ionospheric delay by broadcast ionosphere model (klobuchar model)
 * @param[in] t time (gpst)
 * @param[in] ion iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
 * @param[in] pos receiver position {lat,lon,h} (rad,m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @return ionospheric delay (L1) (m)
*/
extern double ion_KlbModel(GpsTime_t t, const double ion[8], const double *pos, const double *azel);

/**
 * @brief get ionosphere empirical correction
 * @param[in] time time (gpst)
 * @param[in] klb iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}, if input [0], use the default value
 * @param[in] pos receiver position {lat,lon,h} (rad|m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @param[in] ionoopt ionospheric correction option (IONOOPT_???)
 * @param[out] ion ionospheric delay (L1) (m)
 * @param[out] var ionospheric delay (L1) variance (m^2)
 * @return status(0:ok,1:error)
*/
extern int iono_GetIonoEmpiricalCorrection(GpsTime_t time, double klb[8], const double* pos, const double* azel, int ionoopt,
                                double* ion, double* var);

#endif //LOCATION_ENGINE_IONO_EMPIRICAL_MODEL_H
