/**@file        pvt_proc.c
 * @brief       GNSS Position/Velocity/Time Main Process module
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

#include "pvt_proc.h"
#include "pvt_ls.h"
#include "pvt_cf.h"
#include "pvt_kf.h"

static gnss_LsEstimator_t* gpz_LsEstor;

/**
 * @brief Raw measurement qualtiy control
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
static void pvt_qos(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  return;
}

/**
 * @brief Initialze PVT module
 * @return      None
 */
void pvt_init()
{
  gpz_LsEstor = ls_CreateEstimator();

  return;
}

/**
 * @brief Deinitialze PVT module
 * @return      None
 */
void pvt_deinit()
{
  ls_ReleaseEstimator(&gpz_LsEstor);

  return;
}

/**
 * @brief Position Velocity Time Main Process 
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void hpp_Pvt_Process(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  // Qos
  pvt_qos(pz_SatSigMeasCollect);

  // Least Square
  ls_Proc(gpz_LsEstor, pz_SatSigMeasCollect);

  // clock filter
  cf_Proc();

  // Kalman filter
  kf_Proc();

  return;
}
