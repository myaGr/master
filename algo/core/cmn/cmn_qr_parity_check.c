#include "cmn_qr_parity_check.h"
#include <math.h>

//alpha: 0.995
const double CHI_SQUARE_DISTRIBUTION_TABLE[130] =
{
 0.0000393,    0.01,    0.0717,    0.207,     0.412,
     0.676,   0.989,     1.344,    1.735,     2.156,
     2.603,   3.074,     3.565,    4.075,     4.601,
     5.142,   5.697,     6.265,    6.844,     7.434,
     8.034,   8.643,      9.26,    9.886,     10.52,
     11.16,  11.808,     12.461,  13.121,    13.787,
    14.458,  15.134,    15.815,   16.501,    17.192,
    17.887,  18.586,    19.289,   19.996,    20.707,
    21.421,  22.138,    22.859,   23.584,    24.311,
    25.041,  25.775,    26.511,   27.249,    27.991,
    28.735,  29.481,     30.23,   30.981,    31.735,
     32.49,  33.248,    34.008,    34.77,    35.534,
    36.301,  37.068,    37.838,    38.61,    39.383,
    40.158,  40.935,    41.713,   42.494,    43.275,
    44.058,  44.843,    45.629,   46.417,    47.206,
    47.997,  48.788,    49.582,   50.376,    51.172,
    51.969,  52.767,    53.567,   54.368,     55.17,
    55.973,  56.777,    57.582,   58.389,    59.196,
    60.005,  60.815,    61.625,   62.437,     63.25,
    64.063,  64.878,    65.694,    66.51,    67.328,
    68.146,  68.965,    69.785,   70.606,    71.428,
    72.251,  73.075,    73.899,   74.724,     75.55,
    76.377,  77.204,    78.033,   78.862,    79.692,
    80.522,  81.353,    82.185,   83.018,    83.852,
    84.686,   85.52,    86.356,   87.192,    88.029,
    88.866,  89.704,    90.543,   91.382,    92.222
};
/**
 * @brief weight the design matrix and the omc matrix
 * @param[in]  pz_H is the design matrix
 * @param[in]  pz_P is the weight matrix
 * @param[in]  pz_L is the omc (Observation Minus Calculation)
 * @param[out] pz_PH P*H
 * @param[out] pz_PL P*L
 * @return     TRUE represent success and other failed
 */
void cmn_qrWeightObs(const matrix_t* pz_H, const matrix_t* pz_P, const matrix_t* pz_L, matrix_t* pz_PH, matrix_t* pz_PL)
{
  double d_maxWeight = 0.0;
  uint32_t q_rowIndex = 0;
  uint32_t q_colIndex = 0;
  matrix_t* pz_pUnit = matrix_new(pz_P->row, pz_P->col);
  for (q_rowIndex = 0; q_rowIndex < (pz_P->row); ++q_rowIndex)
  {
    if (MAT(pz_P, q_rowIndex, 0) > d_maxWeight)
    {
      d_maxWeight = MAT(pz_P, q_rowIndex, 0);
    }
  }

  for (q_rowIndex = 0; q_rowIndex < (pz_P->row); ++q_rowIndex)
  {
    if (0.0 == MAT(pz_P, q_rowIndex, 0))
    {
      continue;
    }
    MAT(pz_pUnit, q_rowIndex, 0) = MAT(pz_P, q_rowIndex, 0) / d_maxWeight;
  }

  for (q_rowIndex = 0; q_rowIndex < (pz_H->row); ++q_rowIndex)
  {
    if (0.0 == MAT(pz_pUnit, q_rowIndex, 0))
    {
      continue;
    }
    for (q_colIndex = 0; q_colIndex < (pz_H->col); ++q_colIndex)
    {
      if (0.0 == MAT(pz_H, q_rowIndex, q_colIndex))
      {
        continue;
      }
      MAT(pz_PH, q_rowIndex, q_colIndex) = MAT(pz_H, q_rowIndex, q_colIndex) * MAT(pz_pUnit, q_rowIndex, 0);
    }
    MAT(pz_PL, q_rowIndex, 0) = MAT(pz_L, q_rowIndex, 0) * MAT(pz_pUnit, q_rowIndex, 0);
  }
  matrix_free(&pz_pUnit);
  return;
}
/**
 * @brief get the solution of QR decompose
 * @param[in]  pz_Qt is the transpose matrix of Q(n*n)
 * @param[in]  pz_R(n*m) is an upper/right triangular matrix
 * @param[in]  pz_PL is P*L
 * @param[out] pz_deltaX is the delta X of solution
 * @return     void
 */
void solutionPara(const matrix_t* pz_Qt, const matrix_t* pz_R, const matrix_t* pz_PL, matrix_t* pz_deltaX)
{
  uint32_t q_rowNum = pz_R->row;
  uint32_t q_colNum = pz_R->col;
  int32_t q_i = 0;
  int32_t q_j = 0;
  double d_sum = 0.0;
  matrix_t* pz_QtL = matrix_new(q_rowNum, 1);
  for (q_i = 0; q_i < (int32_t)q_rowNum; ++q_i)
  {
    MAT(pz_QtL, q_i, 0) = 0.0;
    for (q_j = 0; q_j < (int32_t)q_rowNum; ++q_j)
    {
      MAT(pz_QtL, q_i, 0) += MAT(pz_Qt, q_i, q_j) * MAT(pz_PL, q_j, 0);
    }
  }

  for (q_i = (int32_t)q_colNum - 1; q_i >= 0; --q_i)
  {
    MAT(pz_deltaX, q_i, 0) = 0.0;
    d_sum = 0.0;
    for (q_j = q_i + 1; q_j < (int32_t)q_colNum; ++q_j)
    {
      d_sum += MAT(pz_R, q_i, q_j) * MAT(pz_deltaX, q_j, 0);
    }
    if (fabs(MAT(pz_R, q_i, q_i)) < 1.0e-18)
    {
      continue;
    }
    MAT(pz_deltaX, q_i, 0) = (MAT(pz_QtL, q_i, 0) - d_sum) / MAT(pz_R, q_i, q_i);
  }
  matrix_free(&pz_QtL);
  return;
}
/**
 * @brief get the information of QR decompose
 * @param[in]  pz_PH is P*H
 * @param[in]  pz_PL is P*L
 * @param[out] pz_lowerQt is the matrix Qt((n-m)*n) of row index start (m-1)
 * @param[out] pz_deltaX is the delta X of solution
 * @return     TRUE represent success and other failed
 */
BOOL cmn_getQrDecompInfor(matrix_t* pz_PH, matrix_t* pz_PL, matrix_t* pz_lowerQt, matrix_t* pz_deltaX)
{
  BOOL z_isSuccess = FALSE;
  uint32_t q_rowNum = pz_PH->row;
  uint32_t q_colNum = pz_PH->col;
  uint32_t q_lowerRowNum = q_rowNum - q_colNum;
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  matrix_t* pz_Qt = NULL;
  matrix_t* pz_R = NULL;
  if (q_lowerRowNum <= 0)
  {
    return FALSE;
  }
  pz_Qt = matrix_new(q_rowNum, q_rowNum);
  pz_R = matrix_new(q_rowNum, q_colNum);
  if (TRUE == matrix_qrParityDecompose(pz_PH, pz_Qt, pz_R))
  {
    for (q_i = q_colNum; q_i < q_rowNum; ++q_i)
    {
      for (q_j = 0; q_j < q_rowNum; ++q_j)
      {
        MAT(pz_lowerQt, q_i - q_colNum, q_j) = MAT(pz_Qt, q_i, q_j);
      }
    }
    solutionPara(pz_Qt, pz_R, pz_PL, pz_deltaX);
    z_isSuccess = TRUE;
  }
  else
  {
    z_isSuccess = FALSE;
  }

  matrix_free(&pz_Qt);
  matrix_free(&pz_R);

  return z_isSuccess;
}
/**
 * @brief calculating the post residual of observations
 * @param[in]  pz_H is the design matrix
 * @param[in]  pz_L is the omc (Observation Minus Calculation)
 * @param[in]  pz_deltaX is the delta X of solution
 * @param[out] pz_postRes is the post residual of observations
 * @return     void
 */
void cmn_calPostRes(const matrix_t* pz_H, const matrix_t* pz_L, const matrix_t* pz_deltaX, matrix_t* pz_postRes)
{
  uint32_t q_rowNum = pz_H->row;
  uint32_t q_colNum = pz_H->col;
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  double d_hX = 0.0;
  for (q_i = 0; q_i < q_rowNum; ++q_i)
  {
    d_hX = 0.0;
    for (q_j = 0; q_j < q_colNum; ++q_j)
    {
      if (0.0 == MAT(pz_H, q_i, q_j))
      {
        continue;
      }
      d_hX += MAT(pz_H, q_i, q_j) * MAT(pz_deltaX, q_j, 0);
    }
    MAT(pz_postRes, q_i, 0) = -(MAT(pz_L, q_i, 0) - d_hX);
  }
  return;
}
/**
 * @brief Chi-square test
 * @param[in]  pz_TL is T*L,T is the matrix Qt((n-m)*n) of row index start (m-1),L is the post residual
 * @param[in]  q_r is n-m, n is the observations number and m is the parameter number
 * @param[in]  d_sigma0 is prior sigma of chi2-test
 * @return     TRUE represent tested pass
 */
BOOL cmn_chi2Test(const matrix_t* pz_TL, uint32_t q_r, double d_sigma0)
{
  BOOL z_testPass = TRUE;
  uint32_t q_rowNum = pz_TL->row;
  uint32_t q_i = 0;
  double d_chiValue = 0.0;
  double d_exmineQuantity = 0.0;
  if (q_r < 130 && q_r>0)
  {
    d_chiValue = CHI_SQUARE_DISTRIBUTION_TABLE[q_r - 1];
    for (q_i = 0; q_i < q_rowNum; ++q_i)
    {
      d_exmineQuantity += MAT(pz_TL, q_i, 0) * MAT(pz_TL, q_i, 0);
    }
    d_exmineQuantity /= (d_sigma0 * d_sigma0);
    if (d_exmineQuantity >= d_chiValue)
    {
      z_testPass = FALSE;
    }
  }
  return z_testPass;
}
/**
 * @brief get the mdules of column vectors by the target colmun index of matrix
 * @param[in]  pz_unitizedMatrix is the target matrix
 * @param[in]  q_colIndex is the target column index of matrix
 * @return     the mdules of column vectors
 */
double cmn_modOfColumnVector(const matrix_t* pz_unitizedMatrix, uint32_t q_colIndex)
{
  double d_mod = 1.0;
  uint32_t q_rowNum = pz_unitizedMatrix->row;
  uint32_t q_colNum = pz_unitizedMatrix->col;
  uint32_t q_i = 0;
  double d_sum = 0.0;
  if (q_colNum <= 0 || q_colIndex >= q_colNum)
  {
    return d_mod;
  }
  for (q_i = 0; q_i < q_rowNum; ++q_i)
  {
    d_sum += MAT(pz_unitizedMatrix, q_i, q_colIndex) * MAT(pz_unitizedMatrix, q_i, q_colIndex);
  }
  d_mod = sqrt(d_sum);
  return d_mod;
}
/**
 * @brief identify the index of gross observations
 * @param[in]  pz_lowerQt is the matrix Qt((n-m)*n) of row index start (m-1)
 * @param[in]  pz_postRes is the post residual of observations
 * @param[in]  pz_TL is T*L,T is the matrix Qt((n-m)*n) of row index start (m-1),L is the post residual
 * @return     the index of gross observation, -1 represent finding gross error observation failure
 */
int32_t cmn_identifyGrossError(const matrix_t* pz_lowerQt, const matrix_t* pz_postRes,const matrix_t* pz_TL)
{
  int32_t q_grossIndex = -1;
  uint32_t q_rowNum = pz_lowerQt->row;
  uint32_t q_colNum = pz_lowerQt->col;
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  matrix_t* pz_testQuantity = matrix_new(q_rowNum, q_colNum);
  double d_minTheta = 999.0;
  double d_TLmod = cmn_modOfColumnVector(pz_TL, 0);
  double d_mod = 0.0;
  double d_dotResult = 0.0;
  double d_temp = 0.0;
  double d_theta = 0.0;
  for (q_i = 0; q_i < q_colNum; ++q_i)
  {
    for (q_j = 0; q_j < q_rowNum; ++q_j)
    {
      MAT(pz_testQuantity, q_j, q_i) = MAT(pz_lowerQt, q_j, q_i) * MAT(pz_postRes, q_i, 0);
    }
  }
  for (q_i = 0; q_i < q_colNum; ++q_i)
  {
    d_dotResult = 0.0;
    for (q_j = 0; q_j < q_rowNum; ++q_j)
    {
      d_dotResult += (MAT(pz_TL, q_j, 0) * MAT(pz_testQuantity, q_j, q_i));
    }
    d_mod = cmn_modOfColumnVector(pz_testQuantity, q_i);
    d_temp = d_dotResult / (d_mod * d_TLmod);
    if (d_temp > 1.0)
    {
      d_temp = 1.0;
    }
    if (d_temp < -1.0)
    {
      d_temp = -1.0;
    }
    d_theta = acos(d_temp);
    if (d_theta < d_minTheta)
    {
      d_minTheta = d_theta;
      q_grossIndex = (int32_t)q_i;
    }
  }
  matrix_free(&pz_testQuantity);
  return q_grossIndex;
}
/**
 * @brief using the method of QR parity check to detect gross error observation according to the post residual
 * @param[in]  d_sigma0 is the prior sigma of chi2-test
 * @param[in]  pz_H is the design matrix
 * @param[in]  pz_P is the weight matrix
 * @param[in]  pz_L is the omc (Observation Minus Calculation)
 * @param[out] pq_grossIndex is the index of gross observation
 * @param[out] pz_deltaX is the delta value of solution
 * @return     TRUE represent success and other failed
 */
QR_check_status cmn_qrParityCheckAcoordingToPostRes(double d_sigma0, const matrix_t* pz_H, const matrix_t* pz_P,
                                         const matrix_t* pz_L, int32_t* pq_grossIndex, matrix_t* pz_deltaX)
{
  QR_check_status u_checkStatus = QR_CHECK_UNKNOW;
  BOOL z_testPass = TRUE;
  uint32_t q_rowNum = pz_H->row;
  uint32_t q_colNum = pz_H->col;
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  uint32_t q_lowerRowNum = q_rowNum - q_colNum;
  matrix_t* pz_PH = matrix_new(q_rowNum, q_colNum);
  matrix_t* pz_PL = matrix_new(pz_L->row, pz_L->col);
  matrix_t* pz_lowerQt = NULL;
  matrix_t* pz_postRes = matrix_new(q_rowNum, 1);
  matrix_t* pz_TL = NULL;
  *pq_grossIndex = -1;
  cmn_qrWeightObs(pz_H, pz_P, pz_L, pz_PH, pz_PL);
  if (q_lowerRowNum > 0)
  {
    pz_lowerQt = matrix_new(q_lowerRowNum, q_rowNum);
    cmn_getQrDecompInfor(pz_PH, pz_PL, pz_lowerQt, pz_deltaX);
    cmn_calPostRes(pz_H, pz_L, pz_deltaX, pz_postRes);
    pz_TL = matrix_new(q_lowerRowNum, 1);
    matrix_mul(pz_lowerQt, pz_postRes, pz_TL);
    z_testPass = cmn_chi2Test(pz_TL, q_lowerRowNum, d_sigma0);
    if (FALSE == z_testPass)
    {
      *pq_grossIndex = cmn_identifyGrossError(pz_lowerQt, pz_postRes, pz_TL);
      if (*pq_grossIndex < 0)
      {
        u_checkStatus = QR_CHECK_IDETIFY_OBS_FAIL;
      }
      else
      {
        u_checkStatus = QR_CHECK_IDETIFY_OBS_SUCC;
      }
    }
    else
    {
      u_checkStatus = QR_CHECK_OBS_NORMAL;
    }
  }
  else
  {
    u_checkStatus = QR_CHECK_OBS_Insufficient;
  }
  matrix_free(&pz_PH);
  matrix_free(&pz_PL);
  matrix_free(&pz_lowerQt);
  matrix_free(&pz_postRes);
  matrix_free(&pz_TL);
  return u_checkStatus;
}