/**@file        pvt_ls.c
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

#include "gnss_type.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "cmn_utils.h"
#include "pvt_ls.h"

typedef enum {
  LS_POS_STATE_X = 0,
  LS_POS_STATE_Y,
  LS_POS_STATE_Z,
  LS_POS_STATE_CLKGPS,
  LS_POS_STATE_CLKGLO,
  LS_POS_STATE_CLKBDS,
  LS_POS_STATE_CLKGAL,
  LS_POS_STATE_MAX,
} ls_PosStateEnumTypeVal;
typedef uint8_t ls_PosStateEnumType;

typedef enum {
  LS_VEL_STATE_X = 0,
  LS_VEL_STATE_Y,
  LS_VEL_STATE_Z,
  LS_VEL_STATE_CLK,
  LS_VEL_STATE_MAX,
} ls_VelStateEnumTypeVal;
typedef uint8_t ls_VelStateEnumType;



static gnss_LsEstimator_t gz_LsEstor = {0};

/**
 * @brief Compute position direction cos
 * @param[in]   d_DC
 * @param[in]   d_range
 * @param[in]   d_xyz
 * @param[in]   pz_satPos
 * @return      None
 */
void ls_ComputePosDirCos(double* d_DC, double* d_range, 
  double* d_xyz, gnss_SatPosVelClk_t* pz_satPos)
{
  double r[3] = {0};
  double rs = 0;

  for (uint8_t i = 0; i < 3; i++)
  {
    r[i] = d_xyz[i] - pz_satPos->d_satPosClk[i];
  }

  rs = vector_norm(r, 3);
  
  for (uint8_t i = 0; i < 3; i++)
  {
    d_DC[i] = r[i] / rs;
  }
  
  if (NULL != d_range)
  { 
    *d_range = rs;
  }

  return;
}


/**
 * @brief Set initialzed position from avg satellite positions
 * @param[in]   pz_LsPcsr
 * @return      None
 */
static void ls_InitPosFromAvgSat(gnss_LsEstimator_t* pz_LsPcsr)
{
  ls_measure_t* pz_LsMeas = &(pz_LsPcsr->z_inputMeas);  
  gnss_coord_t  z_fix_pos = {0};

  for (int i = 0; i < pz_LsMeas->u_UsableSatMeasCount; i++)
  {
    z_fix_pos.d_xyz[0] += pz_LsMeas->pz_UsableSatMeas[i]->z_satPosVelClk.d_satPosClk[0];
    z_fix_pos.d_xyz[1] += pz_LsMeas->pz_UsableSatMeas[i]->z_satPosVelClk.d_satPosClk[1];
    z_fix_pos.d_xyz[2] += pz_LsMeas->pz_UsableSatMeas[i]->z_satPosVelClk.d_satPosClk[2];
  }
  z_fix_pos.d_xyz[0] /= pz_LsMeas->u_UsableSatMeasCount;
  z_fix_pos.d_xyz[1] /= pz_LsMeas->u_UsableSatMeasCount;
  z_fix_pos.d_xyz[2] /= pz_LsMeas->u_UsableSatMeasCount;

  double temp_lla[3] = { 0 };
  gnss_Ecef2Lla(z_fix_pos.d_xyz, temp_lla);
  temp_lla[2] = 0.0;
  gnss_Lla2Ecef(temp_lla, pz_LsPcsr->z_fix_pos.d_xyz);
  pz_LsPcsr->z_fix_pos.u_src = COORD_SRC_AVG_SAT;

  return;
}

static void ls_MeasurementInput(gnss_LsEstimator_t* pz_LsEstor,
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  uint8_t u_UsableSatMeasCount = 0;
  ls_measure_t* pz_inputMeas = &(pz_LsEstor->z_inputMeas);

  for (uint32_t u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; u_constellation++)
  {
    pz_inputMeas->b_MeasValid[u_constellation] = FALSE;
    pz_inputMeas->u_MeasCount[u_constellation] = 0;
  }

  for (uint32_t u_i = 0; u_i < MAX_GNSS_ACTIVE_SAT_NUMBER; u_i++)
  {
    pz_inputMeas->pz_UsableSatMeas[u_i] = NULL;
  }

  for (uint32_t u_i = 0; u_i < pz_SatSigMeasCollect->u_satMeasCount; u_i++)
  {
    uint8_t u_measIdx = pz_SatSigMeasCollect->u_satMeasIdxTable[u_i];
    
    if ((NULL == pz_SatSigMeasCollect->pz_satMeas[u_measIdx]) ||
      (FALSE == pz_SatSigMeasCollect->pz_satMeas[u_measIdx]->z_satPosVelClk.u_valid))
    {
      continue;
    }
    uint8_t u_constellation = pz_SatSigMeasCollect->pz_satMeas[u_measIdx]->u_constellation;

    pz_inputMeas->pz_UsableSatMeas[u_UsableSatMeasCount++] = 
      pz_SatSigMeasCollect->pz_satMeas[u_measIdx];
    pz_inputMeas->b_MeasValid[u_constellation] = TRUE;
    pz_inputMeas->u_MeasCount[u_constellation]++;

  }

  pz_inputMeas->u_UsableSatMeasCount = u_UsableSatMeasCount;

  if ((pz_inputMeas->u_UsableSatMeasCount !=
    vector_u8_sum(&pz_inputMeas->u_MeasCount[0], C_GNSS_MAX)) ||
    (pz_inputMeas->u_UsableSatMeasCount < 4))
  {
    return;
  }

  if (pz_LsEstor->z_fix_pos.u_src == COORD_SRC_NONE) // Need to modify
  {
    ls_InitPosFromAvgSat(pz_LsEstor);
  }
}

/**
 * @brief Least square compute position
 * @param[in]   pz_LsPcsr
 * @return      None
 */
void ls_ComputePos(gnss_LsEstimator_t* pz_LsPcsr)
{
  uint8_t u_ComplementMeasCount = 0;
  uint8_t u_AllMeasCount = 0;

  for (uint8_t u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_QZS; u_constellation++)
  {
    if (!pz_LsPcsr->z_inputMeas.b_MeasValid[u_constellation])
    {
      u_ComplementMeasCount++;
    }
  }

  u_AllMeasCount = pz_LsPcsr->z_inputMeas.u_UsableSatMeasCount + u_ComplementMeasCount;
  if (u_AllMeasCount < LS_POS_STATE_MAX)
  {
    return;
  }

  /* Y = H * X */
  matrix_t* m_H = matrix_new(u_AllMeasCount, LS_POS_STATE_MAX);
  matrix_t* m_Y = matrix_new(u_AllMeasCount, 1);
  matrix_t* m_Ri = matrix_new(LS_POS_STATE_MAX, LS_POS_STATE_MAX);
  matrix_t* m_Gain = matrix_new(LS_POS_STATE_MAX, u_AllMeasCount);
  matrix_t* m_DeltaPosStates = matrix_new(LS_POS_STATE_MAX, 1);

  /* Initilze state matrix */
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_X, 0) = pz_LsPcsr->z_fix_pos.d_xyz[0];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_Y, 0) = pz_LsPcsr->z_fix_pos.d_xyz[1];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_Z, 0) = pz_LsPcsr->z_fix_pos.d_xyz[2];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGPS, 0) = pz_LsPcsr->d_Bias[C_GNSS_GPS];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGLO, 0) = pz_LsPcsr->d_Bias[C_GNSS_GLO];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKBDS, 0) = pz_LsPcsr->d_Bias[C_GNSS_BDS3];
  MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGAL, 0) = pz_LsPcsr->d_Bias[C_GNSS_GAL];
  
  for (uint32_t u_iter = 0; u_iter < 100; u_iter++)
  {
    uint32_t q_measIdx = 0;
    for (q_measIdx = 0; q_measIdx < pz_LsPcsr->z_inputMeas.u_UsableSatMeasCount; q_measIdx++)
    {
      double d_Pos[3] = { pz_LsPcsr->z_fix_pos.d_xyz[0],
                          pz_LsPcsr->z_fix_pos.d_xyz[1],
                          pz_LsPcsr->z_fix_pos.d_xyz[2] };
      double d_DC[3] = { 0 };
      double d_Range = 0.0;
      double d_bias = 0.0;

      gnss_SatelliteMeas_t* pz_SatMeas = pz_LsPcsr->z_inputMeas.pz_UsableSatMeas[q_measIdx];

      ls_ComputePosDirCos(d_DC, &d_Range, d_Pos, &pz_SatMeas->z_satPosVelClk);

      MAT(m_H, q_measIdx, LS_POS_STATE_X) = d_DC[0];
      MAT(m_H, q_measIdx, LS_POS_STATE_Y) = d_DC[1];
      MAT(m_H, q_measIdx, LS_POS_STATE_Z) = d_DC[2];

      switch (pz_SatMeas->u_constellation)
      {
      case C_GNSS_GPS:
        MAT(m_H, q_measIdx, LS_POS_STATE_CLKGPS) = 1.0;
        d_bias = MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGPS, 0);
        break;
      case C_GNSS_GLO:
        MAT(m_H, q_measIdx, LS_POS_STATE_CLKGLO) = 1.0;
        d_bias = MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGLO, 0);
        break;
      case C_GNSS_BDS3:
        MAT(m_H, q_measIdx, LS_POS_STATE_CLKBDS) = 1.0;
        d_bias = MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKBDS, 0);
        break;
      case C_GNSS_GAL:
        MAT(m_H, q_measIdx, LS_POS_STATE_CLKGAL) = 1.0;
        d_bias = MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGAL, 0);
        break;
      case C_GNSS_QZS:
        MAT(m_H, q_measIdx, LS_POS_STATE_CLKGPS) = 1.0;
        d_bias = MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_CLKGPS, 0);
        break;
      }

      MAT(m_Y, q_measIdx, 0) = pz_SatMeas->pz_signalMeas[0]->d_pseudoRange - d_bias - d_Range;
    }

    /* Fill the no measurement constellation clock bias */
    for (uint32_t u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_QZS; u_constellation++)
    {
      if (FALSE == pz_LsPcsr->z_inputMeas.b_MeasValid[u_constellation])
      {
        MAT(m_Y, q_measIdx, 0) = 0.0;
        switch (u_constellation)
        {
        case C_GNSS_GPS:
          MAT(m_H, q_measIdx, LS_POS_STATE_CLKGPS) = 1.0;
          break;
        case C_GNSS_GLO:
          MAT(m_H, q_measIdx, LS_POS_STATE_CLKGLO) = 1.0;
          break;
        case C_GNSS_BDS3:
          MAT(m_H, q_measIdx, LS_POS_STATE_CLKBDS) = 1.0;
          break;
        case C_GNSS_GAL:
          MAT(m_H, q_measIdx, LS_POS_STATE_CLKGAL) = 1.0;
          break;
        default:
          break;
        }
        q_measIdx++;
      }
    }

    /* QR least square
        Y = (Ht*H)^-1 * Ht * X
        H = QR
        Y = R^-1 * Q^T * X */
    if (m_H->row > pz_LsPcsr->m_Q->row)
    {
      matrix_free(&pz_LsPcsr->m_Q);
      pz_LsPcsr->m_Q = matrix_new(m_H->row, LS_POS_STATE_MAX);
    }
    matrix_clean(pz_LsPcsr->m_Q);
    matrix_clean(pz_LsPcsr->m_R);
 
    matrix_qr_decompose(m_H, pz_LsPcsr->m_Q, pz_LsPcsr->m_R);

    matrix_t* m_Qt = matrix_clone(pz_LsPcsr->m_Q);
    matrix_transpose(m_Qt);
    matrix_rightup_inverse(pz_LsPcsr->m_R, m_Ri);

    matrix_mul(m_Ri, m_Qt, m_Gain);
    matrix_mul(m_Gain, m_Y, m_DeltaPosStates);

    matrix_add(pz_LsPcsr->m_PosStates, m_DeltaPosStates);    
    matrix_free(&m_Qt);

    double iterCnvg = matrix_col_vector_norm(m_DeltaPosStates, 0, 0, m_DeltaPosStates->row);
    if (iterCnvg < 1E-3)
    {
      break;
    }
  }
  
  matrix_free(&m_DeltaPosStates);
  matrix_free(&m_Gain);
  matrix_free(&m_Ri);
  matrix_free(&m_H);
  matrix_free(&m_Y);

  return;
}

/**
 * @brief Least square compute velociy
 * @param[in]   pz_LsPcsr
 * @return      None
 */
void ls_ComputeVel(gnss_LsEstimator_t* pz_LsPcsr)
{
  uint8_t u_MeasCount = pz_LsPcsr->z_inputMeas.u_UsableSatMeasCount;
  if (pz_LsPcsr->z_inputMeas.u_UsableSatMeasCount < 4)
  {
    return;
  }

  matrix_t* m_Y = matrix_new(u_MeasCount, 1);
  matrix_t* m_H = matrix_new(u_MeasCount, LS_VEL_STATE_MAX);
 
  for (uint32_t q_measIdx = 0; q_measIdx < pz_LsPcsr->z_inputMeas.u_UsableSatMeasCount; q_measIdx++)
  {
    double d_Pos[3] = { MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_X, 0),
                        MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_Y, 0),
                        MAT(pz_LsPcsr->m_PosStates, LS_POS_STATE_Z, 0) };
    double d_DC[3] = { 0 };
    double d_drift = 0.0;

    gnss_SatelliteMeas_t* pz_SatMeas = pz_LsPcsr->z_inputMeas.pz_UsableSatMeas[q_measIdx];

    ls_ComputePosDirCos(d_DC, NULL, d_Pos, &pz_SatMeas->z_satPosVelClk);
    
    d_drift = pz_SatMeas->z_satPosVelClk.d_satVelClk[3];

    MAT(m_H, q_measIdx, LS_VEL_STATE_X) = d_DC[0];
    MAT(m_H, q_measIdx, LS_VEL_STATE_Y) = d_DC[1];
    MAT(m_H, q_measIdx, LS_VEL_STATE_Z) = d_DC[2];
    MAT(m_H, q_measIdx, LS_VEL_STATE_CLK) = 1.0;
    MAT(m_Y, q_measIdx, 0) = pz_SatMeas->pz_signalMeas[0]->d_doppler - d_drift - 
      pz_SatMeas->z_satPosVelClk.d_satVelClk[3] -
      (d_DC[0] * pz_SatMeas->z_satPosVelClk.d_satVelClk[0] + 
       d_DC[1] * pz_SatMeas->z_satPosVelClk.d_satVelClk[1] +
       d_DC[2] * pz_SatMeas->z_satPosVelClk.d_satVelClk[2]);
  }
  
  matrix_t* m_Ht = matrix_clone(m_H);
  matrix_t* m_HtH = matrix_new(LS_VEL_STATE_MAX, LS_VEL_STATE_MAX);
  matrix_t* m_HtHi = matrix_clone(m_HtH);
  matrix_t* m_Gain = matrix_new(LS_VEL_STATE_MAX, u_MeasCount);
  matrix_transpose(m_Ht);
  
  matrix_mul(m_Ht, m_H, m_HtH);
  matrix_inverse(m_HtH, m_HtHi);
  matrix_mul(m_HtHi, m_Ht, m_Gain);

  matrix_mul(m_Gain, m_Y, pz_LsPcsr->m_VelStates);

  matrix_free(&m_Y);
  matrix_free(&m_H);
  matrix_free(&m_Ht);
  matrix_free(&m_HtH);
  matrix_free(&m_HtHi);
  matrix_free(&m_Gain);
  return;
}

/**
 * @brief Least square compute DOPs
 * @param[in]   pz_LsPcsr
 * @return      None
 */
void ls_ComputeDop(gnss_LsEstimator_t* pz_LsPcsr)
{
  return;
}

/**
 * @brief Create a new least square estimator
 * @param[in]
 * @return      None
 */
gnss_LsEstimator_t* ls_CreateEstimator()
{
  gnss_LsEstimator_t* pz_LsEstor = (gnss_LsEstimator_t*)OS_MALLOC(sizeof(gnss_LsEstimator_t));
  if (NULL == pz_LsEstor)
  {
    return NULL;
  }

  pz_LsEstor->b_init = TRUE;
  pz_LsEstor->m_Q = matrix_new(4, LS_POS_STATE_MAX);
  pz_LsEstor->m_R = matrix_new(LS_POS_STATE_MAX, LS_POS_STATE_MAX);
  pz_LsEstor->m_PosStates = matrix_new(LS_POS_STATE_MAX, 1);
  pz_LsEstor->m_VelStates = matrix_new(LS_VEL_STATE_MAX, 1);

  if ((NULL == pz_LsEstor->m_Q) ||
     (NULL == pz_LsEstor->m_R) ||
    (NULL == pz_LsEstor->m_PosStates) ||
    (NULL == pz_LsEstor->m_VelStates))
  {
    matrix_free(&pz_LsEstor->m_Q);
    matrix_free(&pz_LsEstor->m_R);
    matrix_free(&pz_LsEstor->m_PosStates);
    matrix_free(&pz_LsEstor->m_VelStates);
  }

  return pz_LsEstor;
}

/**
 * @brief Release a least square estimator
 * @param[in]
 * @return      None
 */
void ls_ReleaseEstimator(gnss_LsEstimator_t** pz_LsEstor)
{
  matrix_free(&(*pz_LsEstor)->m_Q);
  matrix_free(&(*pz_LsEstor)->m_R);
  matrix_free(&(*pz_LsEstor)->m_PosStates);
  matrix_free(&(*pz_LsEstor)->m_VelStates);
  OS_FREE(*pz_LsEstor);
  *pz_LsEstor = NULL;
}


/**
 * @brief Least square estimator process
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void ls_Proc(gnss_LsEstimator_t* pz_LsEstor, 
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  ls_MeasurementInput(pz_LsEstor, pz_SatSigMeasCollect);

  ls_ComputePos(pz_LsEstor);
  
  ls_ComputeVel(pz_LsEstor);
  
  ls_ComputeDop(pz_LsEstor);

  return;
}