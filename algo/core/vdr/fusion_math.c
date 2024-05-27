/**@file        fusion_math.c
 * @brief
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_math.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mw_alloc.h"

int8_t double_equal(double a, double b)
{
  if (fabs(a - b) < 1e-15)
  {
    return 1;
  }
  return -1;
}

int8_t float_equal(float a, float b)
{
  if (fabs((double)a - (double)b) < 1e-6)
  {
    return 1;
  }
  return -1;
}

void math_bised_ave_std(float* pf_average, float* pf_std, float f_income, uint32_t u_count)
{
  (*pf_average) = ((float)(u_count - 1) * (*pf_average) + f_income) / (float)u_count;

  if (u_count > 1U)
  {
    (*pf_std) = (*pf_std) * (*pf_std) * (float)(u_count - 2) + (float)u_count / (float)(u_count - 1) * ((*pf_average) - f_income) * ((*pf_average) - f_income);
    (*pf_std) = sqrtf((*pf_std) / (float)(u_count - 1));
  }
  else
  {
    (*pf_std) = 0.0f;
  }
}

void math_calculate_std(float* pf_buf, uint32_t q_len, float* pf_mean, float* pf_std)
{
  float f_meanval = 0.f;
  float f_stdval = 0.f;

  for (uint32_t i = 0; i < q_len; i++)
  {
    f_meanval += pf_buf[i];
  }

  f_meanval /= (float)q_len;

  for (uint32_t i = 0; i < q_len; i++)
  {
    f_stdval = f_stdval + (pf_buf[i] - f_meanval) * (pf_buf[i] - f_meanval);
  }

  f_stdval = sqrtf(f_stdval / (float)q_len);
  *pf_mean = f_meanval;
  *pf_std = f_stdval;
}

void math_calculate_maxmindiff(float* pf_buf, uint32_t q_len, float* pf_diff)
{
  float f_max = pf_buf[0];
  float f_min = pf_buf[0];

  for (uint32_t i = 0; i < q_len; i++)
  {
    if (f_max < pf_buf[i])
    {
      f_max = pf_buf[i];
    }

    if (f_min > pf_buf[i])
    {
      f_min = pf_buf[i];
    }
  }
    
  *pf_diff = f_max - f_min;
}

 /**
  * @brief: calculating mean
  * @param[in]: pf_array - matrix; q_size - size 
  * @param[out]: None
  * @return: average
 */
float math_calculate_mean(float* pf_array, int32_t q_size)
{
  int32_t i;
  float f_sum = 0.f;
  float f_meanvalue = 0.f;
  for (i = 0; i < q_size; i++)
  {
    f_sum = f_sum + pf_array[i];
  }
  f_meanvalue = (f_sum / (float)q_size);
  return f_meanvalue;
}

 /**
  * @brief: calculate eye matrix
  * @param[in]: pf_a - matrix; u_dim - matrix dim
  * @param[out]: eye matrix pf_a
  * @return None
 */
void math_mateye(float* pf_a, uint32_t u_dim)
{
  for (uint32_t i = 0; i < u_dim; i++)
  {
    for (uint32_t j = 0; j < u_dim; j++)
    {
      if (i == j)
      {
        *(pf_a + i * u_dim + j) = 1.f;
      }
      else
      {
        *(pf_a + i * u_dim + j) = 0.f;
      }
    }
  }
}

/**
 * @brief: calculate vector cross product c=aXb
 * @param[in]: pf_a - 3D vector a; pf_b - 3D vector b;
 * @param[out]: pf_c - 3D vector c
 * @return: None
*/
void math_cross_product(float pf_a[3], float pf_b[3], float pf_c[3])
{
  pf_c[0] = pf_a[1] * pf_b[2] - pf_b[1] * pf_a[2];
  pf_c[1] = pf_b[0] * pf_a[2] - pf_a[0] * pf_b[2];
  pf_c[2] = pf_a[0] * pf_b[1] - pf_b[0] * pf_a[1];
}

/*
* @brief: generate skew-symmetric matrix
* @param[in]: pf_vct - 3D Vector
* @param[out]: pf_mat - 3*3 matrix
* @return: None
*
*/
void math_skewsym(float pf_vct[3], float pf_mat[3][3])
{
  pf_mat[0][0] = 0.f;
  pf_mat[0][1] = -pf_vct[2];
  pf_mat[0][2] = pf_vct[1];

  pf_mat[1][0] = pf_vct[2];
  pf_mat[1][1] = 0.f;
  pf_mat[1][2] = -pf_vct[0];

  pf_mat[2][0] = -pf_vct[1];
  pf_mat[2][1] = pf_vct[0];
  pf_mat[2][2] = 0.f;
}

/*
* @brief: add two matrix (C = A+B)
* @param[in]: pf_a - matrix A; pf_b - matrix B; u_row - matrix rows; u_col - matrix cols
* @param[out]: pf_c - matrix C
* @return: None
*
*/
void math_matadd(float pf_a[], float pf_b[], uint32_t u_row, uint32_t u_col, float pf_c[]) 
{
  for (uint32_t i = 0; i < u_row; i++)
  {
    for (uint32_t j = 0; j < u_col; j++)
    {
      pf_c[i * u_col + j] = pf_a[i * u_col + j] + pf_b[i * u_col + j];
    }
  }
}

/*
* @brief: matrix subtraction(A[m][n]-B[m][n]=C[m][n])
* @param[in]:pf_a - matrix A; pf_b - matrix B; u_row - matrix rows; u_col - matrix cols
* @param[out]: pf_c - matrix C
* @return: None
*
*/
void math_matsub(float pf_a[], float pf_b[], uint32_t u_row, uint32_t u_col, float pf_c[])
{
  for (uint32_t i = 0; i < u_row; i++)
  {
    for (uint32_t j = 0; j < u_col; j++)
    {
      pf_c[i * u_col + j] = pf_a[i * u_col + j] - pf_b[i * u_col + j];
    }
  }
}

/*
* @brief: matrix transpose(B = A(T))
* @param[in]: pf_a - matrix A; u_row - matrix rows; u_col - matrix cols
* @param[out]: pf_b - matrix B
* @return: None
*
*/
void math_mattrns(float pf_a[], uint32_t u_row, uint32_t u_col, float pf_b[])
{
  for (uint32_t i = 0; i < u_col; i++)
  {
    for (uint32_t j = 0; j < u_row; j++)
    {
      pf_b[i * u_row + j] = pf_a[j * u_col + i];
    }
  }
}

/*
* @brief: matrix multiply (A[m][n]*B[n][k]=C[m][k])
* @param[in]: pf_a - matrix A; pf_b - matrix B; u_arow - matrix arows; u_acol - matrix acols; u_bcol - matrix bcols
* @param[out]: pf_c - matrix C
* @return: None
*
*/
void math_matmul(float pf_a[], float pf_b[], uint32_t u_arow, uint32_t u_acol, uint32_t u_bcol, float pf_c[])
{
  uint32_t i, j, t, u;
  for (i = 0; i < u_arow; i++)
  {
    for (j = 0; j < u_bcol; j++)
    {
      u = i * u_bcol + j;
      pf_c[u] = 0.f;
      for (t = 0; t < u_acol; t++)
      {
        pf_c[u] = pf_c[u] + pf_a[i * u_acol + t] * pf_b[t * u_bcol + j];
      }
    }
  }
}

/*
* @brief: muitiply matrix with constant
* @param[in]: pf_a - matrix A; f_b - coef; u_row - matrix rows; u_col - matrix cols
* @param[out]: pf_c - matrix C
* @return: None
*
*/
void math_matmul_const(float pf_a[], float f_b, uint32_t u_row, uint32_t u_col, float pf_c[])
{
  uint32_t i, j;
  for (i = 0; i < u_row; i++)
  {
    for (j = 0; j < u_col; j++)
    {
      pf_c[i * u_col + j] = pf_a[i * u_col + j] * f_b;
    }
  }
}

/*
* @brief: matrix transpose(B = A(T))
* @param[in]: f_a - matrix A; u_row - matrix rows; u_col - matrix cols
* @param[out]: f_b - matrix B
* @return: None
*
*/
void math_matrix_transpose(float f_a[], uint32_t u_row, uint32_t u_col, float f_b[])
{
  uint32_t i = 0;
  uint32_t j = 0;
  for (i = 0; i < u_col; i++)
  {
    for (j = 0; j < u_row; j++)
    {
      f_b[i * u_row + j] = f_a[j * u_col + i];
    }
  }
}

/*
* @brief: matrix square transpose
* @param[in]: u_rows - rows
* @param[out]: f_m - result matrix
* @return: None
*
*/
void math_square_transpose(float f_m[], uint32_t u_rows)
{
  if (u_rows > 1)
  {
    float f_stmp;
    uint32_t i, j;

    for (i = 1; i < u_rows; i++)
    {
      for (j = 0; j < i; j++)
      {
        f_stmp = f_m[i * u_rows + j];
        f_m[i * u_rows + j] = f_m[j * u_rows + i];
        f_m[j * u_rows + i] = f_stmp;
      }
    }
  }
}

/*
* @brief: copy matrix
* @input: f_b - matrix; u_row - rows; u_col - cols
* @output: f_a - result matrix
* @return: None
*
*/
void math_matcopy(float f_a[], float f_b[], uint32_t u_row, uint32_t u_col)
{
  uint32_t i, j;
  for (i = 0; i < u_row; i++)
  {
    for (j = 0; j < u_col; j++)
    {
      f_a[i * u_col + j] = f_b[i * u_col + j];
    }
  }
}

/*
* @brief: matrix cholesky decompositon, and record lower triangular matrix
* @input: u_cols - input matrix cols
* @output: pf_m - result matrix
* @return: 0:pass -1:fail
*
*/
int8_t math_choleskydc(float* pf_m, uint32_t u_cols)
{
  int8_t s_val = 0;
  uint32_t i, j;
  int k;
  float f_sum;
  float* pf_p = NULL;

  if (u_cols <= 0)
  {
    s_val = -1;
  }
  else
  {
    pf_p = (float*)OS_MALLOC(sizeof(float) * u_cols * 1);
  }

  if (pf_p != NULL)
  {
    for (i = 0; i < u_cols; i++)
    {
      for (j = i; j < u_cols; j++)
      {
        f_sum = pf_m[i * u_cols + j];

        for (k = i - 1; k >= 0; k--)
        {
          f_sum -= pf_m[i * u_cols + k] * pf_m[j * u_cols + k];
        }

        if (i == j)
        {
          if (f_sum <= 0.f)
          {
            s_val = -1;
            break;
          }
          pf_p[i] = sqrtf(f_sum);
        }
        else
        {
          pf_m[j * u_cols + i] = f_sum / pf_p[i];
        }
      }
    }

    for (i = 0; i < u_cols; i++)
    {
      pf_m[i * u_cols + i] = pf_p[i];
    }

    for (i = 0; i < u_cols; i++)
    {
      for (j = i + 1; j < u_cols; j++)
      {
        pf_m[i * u_cols + j] = 0.f;
      }
    }
    math_square_transpose(pf_m, u_cols);
  }
  else
  {
    s_val = -1;
  }
  OS_FREE(pf_p);

  return s_val;
}

void math_matextract(float f_a[], uint32_t pn, float f_b[], uint32_t u_cm, uint32_t u_cn, uint32_t u_startm, uint32_t u_endn)
{
  uint32_t ii, jj;
  for (ii = 0; ii < u_cm; ii++)
  {
    for (jj = 0; jj < u_cn; jj++)
    {
      f_b[ii * u_cn + jj] = f_a[(ii + u_startm) * pn + jj + u_endn];
    }
  }
}

/*
* @brief: matrix transpose
* @input: dim - matrix dim; row - matrix row; col - matrix col
* @output: m - result matrix
* @return: None
*
*/
void MatTransform(float* m, int dim, int* row, int* col)
{
  int kcount, zcount;
  for (kcount = dim - 1; kcount >= 0; kcount--)
  {
    if (col[kcount] != kcount)
    {
      for (zcount = 0; zcount < dim; zcount++)
      {
        float dTmp = m[kcount * dim + zcount];
        m[kcount * dim + zcount] = m[col[kcount] * dim + zcount];
        m[col[kcount] * dim + zcount] = dTmp;
      }
    }

    if (row[kcount] != kcount)
    {
      for (zcount = 0; zcount < dim; zcount++)
      {
        float dTmp = m[zcount * dim + kcount];
        m[zcount * dim + kcount] = m[zcount * dim + row[kcount]];
        m[zcount * dim + row[kcount]] = dTmp;
      }
    }
  }
}

/*
* @brief: matrix inversion
* @input:u_dim - matrix dim
* @outpu:pf_m - result matrix; pf_det - determinant
* @return: 0:pass 1:fail
*
*/
int8_t math_matinv(float* pf_m, uint32_t u_dim, float* pf_det)
{
  int8_t s_val = 0;
  float	fMax = 0.f;
  int fSign = 1;
  int* IndexRow = NULL;
  int* IndexCol = NULL;
  uint32_t icount = 0;
  uint32_t jcount = 0;
  uint32_t kcount = 0;
  uint32_t zcount = 0;
  int matSize = u_dim * u_dim;

  IndexRow = (int*)OS_MALLOC((unsigned int)matSize * (sizeof(int)));
  IndexCol = (int*)OS_MALLOC((unsigned int)matSize * (sizeof(int)));

  if (IndexRow != NULL && IndexCol != NULL)
  {
    *pf_det = 1.f;

    memset(IndexRow, 0, (unsigned int)matSize * (sizeof(int)));
    memset(IndexCol, 0, (unsigned int)matSize * (sizeof(int)));

    for (kcount = 0; kcount < u_dim; kcount++)
    {
      /* 1. get main element */
      fMax = 0.f;
      for (icount = kcount; icount < u_dim; icount++)
      {
        for (jcount = kcount; jcount < u_dim; jcount++)
        {
          float dTmp = fabsf(pf_m[icount * u_dim + jcount]);
          if (dTmp > fMax)
          {
            fMax = dTmp;
            IndexRow[kcount] = icount;
            IndexCol[kcount] = jcount;
          }
        }
      }
      /* 2. if wrong matrix return */
      if (fMax < 1e-5)
      {
        s_val = -1;
        break;
      }
      /* 3. convert rows */
      if (IndexRow[kcount] != kcount)
      {
        fSign = -fSign;
        for (zcount = 0; zcount < u_dim; zcount++)
        {
          float dTmp = pf_m[kcount * u_dim + zcount];
          pf_m[kcount * u_dim + zcount] = pf_m[IndexRow[kcount] * u_dim + zcount];
          pf_m[IndexRow[kcount] * u_dim + zcount] = dTmp;
        }
      }
      /* convert cols */
      if (IndexCol[kcount] != kcount)
      {
        fSign = -fSign;
        for (zcount = 0; zcount < u_dim; zcount++)
        {
          float dTmp = pf_m[zcount * u_dim + kcount];
          pf_m[zcount * u_dim + kcount] = pf_m[zcount * u_dim + IndexCol[kcount]];
          pf_m[zcount * u_dim + IndexCol[kcount]] = dTmp;
        }
      }
      /* get determinant */
      *pf_det = (*pf_det) * pf_m[kcount * u_dim + kcount];

      /* 4. compute inverse */
      pf_m[kcount * u_dim + kcount] = 1.0f / pf_m[kcount * u_dim + kcount];

      for (jcount = 0; jcount < u_dim; jcount++)
      {
        if (jcount != kcount)
        {
          pf_m[kcount * u_dim + jcount] *= pf_m[kcount * u_dim + kcount];
        }
      }

      for (icount = 0; icount < u_dim; icount++)
      {
        if (icount != kcount)
        {
          for (jcount = 0; jcount < u_dim; jcount++)
          {
            if (jcount != kcount)
            {
              pf_m[icount * u_dim + jcount] -= (pf_m[icount * u_dim + kcount] * pf_m[kcount * u_dim + jcount]);
            }
          }
        }
      }

      for (icount = 0; icount < u_dim; icount++)
      {
        if (icount != kcount)
        {
          pf_m[icount * u_dim + kcount] *= (-pf_m[kcount * u_dim + kcount]);
        }
      }

    }

    if (s_val >= 0)
    {
      MatTransform(pf_m, u_dim, IndexRow, IndexCol);

      *pf_det *= (float)fSign;

      s_val = 0;
    }
  }
  OS_FREE(IndexRow);
  OS_FREE(IndexCol);
  return s_val;
}

/*
* @brief: Cholesky decomposition and inversion
* @input: u_dim - matrix dim
* @output: pf_matinv - result matrix
* @return: success flag
*/
uint8_t math_mat_calinverse(float* pf_matinv, uint32_t u_dim)
{
  float MatDet = 0.0f;
  float* InvTran = NULL;
  float* MatForInv_Temp = NULL;
  uint32_t ret_inv = 0;

  InvTran = (float*)OS_MALLOC(u_dim * u_dim * sizeof(float));
  MatForInv_Temp = (float*)OS_MALLOC(u_dim * u_dim * sizeof(float));

  if (InvTran != NULL && MatForInv_Temp != NULL)
  {
    memset(InvTran, 0, (sizeof(float) * (u_dim) * (u_dim)));
    memset(MatForInv_Temp, 0, (sizeof(float) * (u_dim) * (u_dim)));

    if ((math_choleskydc(pf_matinv, u_dim) >= (int8_t)0) && (math_matinv(pf_matinv, u_dim, &MatDet) >= (int8_t)0))
    {
      math_matcopy(InvTran, pf_matinv, u_dim, u_dim);
      math_square_transpose(InvTran, u_dim);

      math_matmul(pf_matinv, InvTran, u_dim, u_dim, u_dim, MatForInv_Temp);
      math_matcopy(pf_matinv, MatForInv_Temp, u_dim, u_dim);

      ret_inv = 1;
    }
    else
    {
      memset(pf_matinv, 0, (sizeof(float) * (u_dim) * (u_dim)));
    }
  }
  OS_FREE(InvTran);
  OS_FREE(MatForInv_Temp);
  return ret_inv;
}
