/**@file        sd_task.h
 * @brief       Satellite Datebase Process
 * @details     1. Ephemeris/Almanac calculate
 *              2. SSR data calculate
 * @author      caizhijie
 * @date        2022/04/26
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SD_TASK_H__
#define __SD_TASK_H__

#include "cmn_def.h"
#include "gnss_type.h"
#include "os_mutex.h"

BEGIN_DECL

typedef struct {
  uint8_t         b_init;
  mutex_t         z_mtx;
  uint64_t        t_gnssEphMask[C_GNSS_MAX];
  gnss_Ephemeris_t* pz_gnssEph[ALL_GNSS_SYS_SV_NUMBER][GNSS_EPHEMERIS_CACHE_NUM];
} sd_GnssEphemerisPool_t;

typedef struct {
  uint8_t                        b_init;
  mutex_t                        z_mtx;
  uint64_t                       t_gnssSatPvtPolyMask[C_GNSS_MAX];
  gnss_SatPosVelClkPolynomial_t* pz_gnssSatPvtPoly[ALL_GNSS_SYS_SV_NUMBER];
} sd_GnssSatPosVelClkPolynomialPool_t;

END_DECL

#endif
