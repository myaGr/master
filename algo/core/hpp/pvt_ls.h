/**@file        pvt_ls.h
 * @brief       GNSS least square
 * @details
 * @author      caizhijie
 * @date        2022/06/13
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/13  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __PVT_LS_H__
#define __PVT_LS_H__

#include "cmn_def.h"
#include "cmn_utils.h"
#include "gnss_type.h"

BEGIN_DECL

typedef struct {
  uint8_t  b_MeasValid[C_GNSS_MAX];
  uint8_t  u_MeasCount[C_GNSS_MAX];
  uint8_t  u_UsableSatMeasCount;
  gnss_SatelliteMeas_t* pz_UsableSatMeas[MAX_GNSS_ACTIVE_SAT_NUMBER];
} ls_measure_t;

typedef struct {
  uint8_t       b_init;
  GpsTime_t     z_fixTime;
  matrix_t*     m_PosStates;
  matrix_t*     m_VelStates;
  matrix_t*     m_Q;
  matrix_t*     m_R;
  gnss_coord_t  z_fix_pos;
  gnss_coord_t  z_fix_vel;
  double        d_Bias[C_GNSS_MAX];
  ls_measure_t  z_inputMeas;
} gnss_LsEstimator_t;

/**
 * @brief Create a new least square estimator
 * @param[in]
 * @return      None
 */
gnss_LsEstimator_t* ls_CreateEstimator();

/**
 * @brief Release a least square estimator
 * @param[in]
 * @return      None
 */
void ls_ReleaseEstimator(gnss_LsEstimator_t** pz_LsEstor);

/**
 * @brief Least square estimate process
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void ls_Proc(gnss_LsEstimator_t* pz_LsEstor,
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

END_DECL

#endif
