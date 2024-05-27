#ifndef __CMN_SPARSE_MATRIX_MULTIPLY_H__
#define __CMN_SPARSE_MATRIX_MULTIPLY_H__
#include "cmn_def.h"
BEGIN_DECL
typedef struct
{
  int16_t w_rowIndex;  /* the row index of matrix element */
  int16_t w_columIndex;/* the column index of matrix element */
  double  d_value;     /* the value of matrix element */
}gnss_tripleDoubleEle;

typedef struct
{
  int16_t w_rowIndex;  /* the row index of matrix element */
  int16_t w_columIndex;/* the column index of matrix element */
  float   f_value;     /* the value of matrix element */
}gnss_tripleFloatEle;

typedef struct
{
  uint16_t w_rowNum;                    /* the row number of matrix */
  uint16_t w_colNum;                    /* the column number of matrix */
  uint32_t q_eleNum;                    /* the number of sparse matrix */
  gnss_tripleDoubleEle* pz_tripleEleSet;/* the elemments of sparse matrix */
}gnss_sparseDoubleMatrix;

typedef struct
{
  uint16_t w_rowNum;                    /* the row number of matrix */
  uint16_t w_colNum;                    /* the column number of matrix */
  uint32_t q_eleNum;                    /* the number of sparse matrix */
  gnss_tripleFloatEle* pz_tripleEleSet;/* the elemments of sparse matrix */
}gnss_sparseFloatMatrix;

/**
 * @brief initilize the elements of sparse matrix,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_initTripleDoubleEle(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief initilize the elements of sparse matrix,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_initTripleFloatEle(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief ascending sort the elements of sparse matrix by column index,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleDoubleEleByColumnIndex(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief ascending sort the elements of sparse matrix by column index,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleFloatEleByColumnIndex(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief ascending sort the elements of sparse matrix by row index,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleDoubleEleByRowIndex(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief ascending sort the elements of sparse matrix by row index,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleFloatEleByRowIndex(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum);
/**
 * @brief sparse matrix A multiply the full matrix B, C=A*B,the value type is double
 * @param[in]    pz_leftMatrix represent sparse matrix A
 * @param[in]    pd_rightMatrix represent the full matrix B
 * @param[in]    w_rightRowNum is the row number of full matrix B
 * @param[in]    w_rightColNum is the column number of full matrix B
 * @param[out]   pd_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_sparseMultiplyFullDouble(const gnss_sparseDoubleMatrix* pz_leftMatrix,
  const double* pd_rightMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, double* pd_result);
/**
 * @brief sparse matrix A multiply the full matrix B, C=A*B,the value type is float
 * @param[in]    pz_leftMatrix represent sparse matrix A
 * @param[in]    pf_rightMatrix represent the full matrix B
 * @param[in]    w_rightRowNum is the row number of full matrix B
 * @param[in]    w_rightColNum is the column number of full matrix B
 * @param[out]   pf_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_sparseMultiplyFullFloat(const gnss_sparseFloatMatrix* pz_leftMatrix,
  const float* pf_rightMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, float* pf_result);

/**
 * @brief sparse matrix A multiply the symmetry matrix B, C=A*B,the value type is double
 * @param[in]    pz_leftMatrix represent sparse matrix A
 * @param[in]    pd_symMatrix represent the symmetry matrix B
 * @param[in]    w_rightRowNum is the row number of symmetry matrix B
 * @param[in]    w_rightColNum is the column number of symmetry matrix B
 * @param[out]   pd_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_sparseMultiplySymmetryDouble(const gnss_sparseDoubleMatrix* pz_leftMatrix,
  const double* pd_symMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, double* pd_result);

/**
 * @brief sparse matrix A multiply the symmetry matrix B, C=A*B,the value type is float
 * @param[in]    pz_leftMatrix represent sparse matrix A
 * @param[in]    pf_symMatrix represent the symmetry matrix B
 * @param[in]    w_rightRowNum is the row number of symmetry matrix B
 * @param[in]    w_rightColNum is the column number of symmetry matrix B
 * @param[out]   pf_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_sparseMultiplySymmetryFloat(const gnss_sparseFloatMatrix* pz_leftMatrix,
  const float* pf_symMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, float* pf_result);

/**
 * @brief full matrix A multiply sparse matrix B, C=A*B,the value type is double
 * @param[in]    pd_leftMatrix represent full matrix A
 * @param[in]    w_leftRowNum is the row number of full matrix A
 * @param[in]    w_leftColNum is the column number of full matrix A
 * @param[in]    pz_rightMatrix represent the sparse matrix B
 * @param[out]   pd_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_fullMultiplySparseDouble(const double* pd_leftMatrix, uint16_t w_leftRowNum,
  uint16_t w_leftColNum, const gnss_sparseDoubleMatrix* pz_rightMatrix, double* pd_result);
/**
 * @brief full matrix A multiply sparse matrix B, C=A*B,the value type is float
 * @param[in]    pf_leftMatrix represent full matrix A
 * @param[in]    w_leftRowNum is the row number of full matrix A
 * @param[in]    w_leftColNum is the column number of full matrix A
 * @param[in]    pz_rightMatrix represent the sparse matrix B
 * @param[out]   pf_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_fullMultiplySparseFloat(const float* pf_leftMatrix, uint16_t w_leftRowNum,
  uint16_t w_leftColNum, const gnss_sparseFloatMatrix* pz_rightMatrix, float* pf_result);

/**
 * @brief full matrix A multiply sparse matrix B, C=A*B,the value type is double
 *        only calculating the elements of lower triangle
 * @param[in]    pd_leftMatrix represent full matrix A
 * @param[in]    w_leftRowNum is the row number of full matrix A
 * @param[in]    w_leftColNum is the column number of full matrix A
 * @param[in]    pz_rightMatrix represent the sparse matrix B
 * @param[out]   pd_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_fullMultiplySparseDoubleSymmetry(const double* pd_leftMatrix, uint16_t w_leftRowNum,
  uint16_t w_leftColNum, const gnss_sparseDoubleMatrix* pz_rightMatrix, double* pd_result);
/**
 * @brief full matrix A multiply sparse matrix B, C=A*B,the value type is float
 *        only calculating the elements of lower triangle
 * @param[in]    pf_leftMatrix represent full matrix A
 * @param[in]    w_leftRowNum is the row number of full matrix A
 * @param[in]    w_leftColNum is the column number of full matrix A
 * @param[in]    pz_rightMatrix represent the sparse matrix B
 * @param[out]   pf_result is the result of A*B, pd_result must be set to zero externally
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t gnss_fullMultiplySparseFloatSymmetry(const float* pf_leftMatrix, uint16_t w_leftRowNum,
  uint16_t w_leftColNum, const gnss_sparseFloatMatrix* pz_rightMatrix, float* pf_result);

END_DECL
#endif