#include "cmn_sparse_matrix_multiply.h"
#include "mw_alloc.h"
#include "gnss_type.h"
/**
 * @brief initilize the elements of sparse matrix,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_initTripleDoubleEle(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    if (NULL != pz_tripleEleSet)
    {
        for (q_i = 0; q_i < q_eleNum; ++q_i)
        {
            pz_tripleEleSet[q_i].w_rowIndex = -1;
            pz_tripleEleSet[q_i].w_columIndex = -1;
            pz_tripleEleSet[q_i].d_value = 0.0;
        }
    }
    return;
}
/**
 * @brief initilize the elements of sparse matrix,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_initTripleFloatEle(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    if (NULL != pz_tripleEleSet)
    {
        for (q_i = 0; q_i < q_eleNum; ++q_i)
        {
            pz_tripleEleSet[q_i].w_rowIndex = -1;
            pz_tripleEleSet[q_i].w_columIndex = -1;
            pz_tripleEleSet[q_i].f_value = 0.0;
        }
    }
    return;
}
/**
 * @brief compare the row index between base and target
 * @param[in]    w_baseRowIndex represent the row index of base element
 * @param[in]    w_baseColumIndex represent the column index of base element
 * @param[in]    w_targetRowIndex represent the row index of target element
 * @param[in]    w_targetColumIndex represent the column index of target element
 * @return       the compare result based on row index prior,the return value 1 represent base less than target
 */
uint8_t gnss_compareTripleEleByRow(int16_t w_baseRowIndex, int16_t w_baseColumIndex,
    int16_t w_targetRowIndex, int16_t w_targetColumIndex)
{
    uint8_t u_result = 0;
    if (w_baseRowIndex < w_targetRowIndex)
    {
        u_result = 1;
    }
    else if (w_baseRowIndex > w_targetRowIndex)
    {
        u_result = 0;
    }
    else if (w_baseRowIndex == w_targetRowIndex)
    {
        if (w_baseColumIndex <= w_targetColumIndex)
        {
            u_result = 1;
        }
        else
        {
            u_result = 0;
        }
    }
    return u_result;
}
/**
 * @brief compare the row index between base and target
 * @param[in]    w_baseRowIndex represent the row index of base element
 * @param[in]    w_baseColumIndex represent the column index of base element
 * @param[in]    w_targetRowIndex represent the row index of target element
 * @param[in]    w_targetColumIndex represent the column index of target element
 * @return       the compare result based on column index prior,the return value 1 represent base less than target
 */
uint8_t gnss_compareTripleEleByCol(int16_t w_baseRowIndex, int16_t w_baseColumIndex,
    int16_t w_targetRowIndex, int16_t w_targetColumIndex)
{
    uint8_t u_result = 0;
    if (w_baseColumIndex < w_targetColumIndex)
    {
        u_result = 1;
    }
    else if (w_baseColumIndex > w_targetColumIndex)
    {
        u_result = 0;
    }
    else if (w_baseColumIndex == w_targetColumIndex)
    {
        if (w_baseRowIndex <= w_targetRowIndex)
        {
            u_result = 1;
        }
        else
        {
            u_result = 0;
        }
    }
    return u_result;
}
/**
 * @brief ascending sort the elements of sparse matrix by row index,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleDoubleEleByRowIndex(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    gnss_tripleDoubleEle z_tripTemp = { 0 };
    for (q_i = 0; q_i < q_eleNum - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_eleNum; ++q_j)
        {
            if (1 == gnss_compareTripleEleByRow(pz_tripleEleSet[q_i].w_rowIndex, pz_tripleEleSet[q_i].w_columIndex,
                pz_tripleEleSet[q_j].w_rowIndex, pz_tripleEleSet[q_j].w_columIndex))
            {
                continue;
            }
            z_tripTemp = pz_tripleEleSet[q_i];
            pz_tripleEleSet[q_i] = pz_tripleEleSet[q_j];
            pz_tripleEleSet[q_j] = z_tripTemp;
        }
    }
    return;
}
/**
 * @brief ascending sort the elements of sparse matrix by row index,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleFloatEleByRowIndex(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    gnss_tripleFloatEle z_tripTemp = { 0 };
    for (q_i = 0; q_i < q_eleNum - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_eleNum; ++q_j)
        {
            if (pz_tripleEleSet[q_j].w_rowIndex < pz_tripleEleSet[q_i].w_rowIndex)
            {
                z_tripTemp = pz_tripleEleSet[q_i];
                pz_tripleEleSet[q_i] = pz_tripleEleSet[q_j];
                pz_tripleEleSet[q_j] = z_tripTemp;
            }
        }
    }
    return;
}
/**
 * @brief ascending sort the elements of sparse matrix by column index,the value type is double
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleDoubleEleByColumnIndex(gnss_tripleDoubleEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    gnss_tripleDoubleEle z_tripTemp = { 0 };
    for (q_i = 0; q_i < q_eleNum - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_eleNum; ++q_j)
        {
            if (1 == gnss_compareTripleEleByCol(pz_tripleEleSet[q_i].w_rowIndex, pz_tripleEleSet[q_i].w_columIndex,
                pz_tripleEleSet[q_j].w_rowIndex, pz_tripleEleSet[q_j].w_columIndex))
            {
                continue;
            }
            z_tripTemp = pz_tripleEleSet[q_i];
            pz_tripleEleSet[q_i] = pz_tripleEleSet[q_j];
            pz_tripleEleSet[q_j] = z_tripTemp;
        }
    }
    return;
}
/**
 * @brief ascending sort the elements of sparse matrix by column index,the value type is float
 * @param[in]    pz_tripleEleSet represent the elements of sparse matrix
 * @param[in]    q_eleNum represent the number of sparse matrix elements
 * @return       void
 */
void gnss_sortTripleFloatEleByColumnIndex(gnss_tripleFloatEle* pz_tripleEleSet, uint32_t q_eleNum)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    gnss_tripleFloatEle z_tripTemp = { 0 };
    for (q_i = 0; q_i < q_eleNum - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_eleNum; ++q_j)
        {
            if (pz_tripleEleSet[q_j].w_columIndex < pz_tripleEleSet[q_i].w_columIndex)
            {
                z_tripTemp = pz_tripleEleSet[q_i];
                pz_tripleEleSet[q_i] = pz_tripleEleSet[q_j];
                pz_tripleEleSet[q_j] = z_tripTemp;
            }
        }
    }
    return;
}
/**
 * @brief get number of rows in the matrix,the value type is double
 * @param[in]    pz_sparseMatrix represent sparse matrix A
 * @param[out]   pw_pos is the number of rows in the matrix
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t findNonZeroRowPosDouble(const gnss_sparseDoubleMatrix* pz_sparseMatrix, uint16_t* pw_pos)
{
    uint16_t w_nRows = (pz_sparseMatrix->w_rowNum);
    uint32_t q_eleNum = (pz_sparseMatrix->q_eleNum);
    uint32_t q_i = 0;
    int16_t w_rowIndex = 0;
    uint16_t* pw_num = NULL;
    for (q_i = 0; q_i < q_eleNum; ++q_i)
    {
        w_rowIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_rowIndex;
        if (w_rowIndex < 0 || w_rowIndex >= w_nRows)
        {
            return 0;
        }
    }
    pw_num = (uint16_t*)OS_MALLOC(w_nRows * sizeof(uint16_t));
    if (NULL != pw_num)
    {
        for (q_i = 0; q_i < w_nRows; ++q_i)
        {
            pw_num[q_i] = pw_pos[q_i] = 0;
        }
        pw_pos[w_nRows] = 0;
        for (q_i = 0; q_i < q_eleNum; q_i++)
        {
            w_rowIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_rowIndex;
            pw_num[w_rowIndex]++;
        }
        for (q_i = 1; q_i <= w_nRows; q_i++)
        {
            pw_pos[q_i] = pw_pos[q_i - 1] + pw_num[q_i - 1];
        }
        OS_FREE(pw_num);
    }
    return 1;
}
/**
 * @brief get number of rows in the matrix,the value type is float
 * @param[in]    pz_sparseMatrix represent sparse matrix A
 * @param[out]   pw_pos is the number of rows in the matrix
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t findNonZeroRowPosFloat(const gnss_sparseFloatMatrix* pz_sparseMatrix, uint16_t* pw_pos)
{
    uint16_t w_nRows = (pz_sparseMatrix->w_rowNum);
    uint32_t q_eleNum = (pz_sparseMatrix->q_eleNum);
    uint32_t q_i = 0;
    int16_t w_rowIndex = 0;
    uint16_t* pw_num = NULL;
    for (q_i = 0; q_i < q_eleNum; ++q_i)
    {
        w_rowIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_rowIndex;
        if (w_rowIndex < 0 || w_rowIndex >= w_nRows)
        {
            return 0;
        }
    }
	
    pw_num = (uint16_t*)OS_MALLOC(w_nRows * sizeof(uint16_t));
    if (NULL != pw_num)
    {
        for (q_i = 0; q_i < w_nRows; ++q_i)
        {
            pw_num[q_i] = pw_pos[q_i] = 0;
        }
        pw_pos[w_nRows] = 0;
        for (q_i = 0; q_i < q_eleNum; q_i++)
        {
            w_rowIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_rowIndex;
            pw_num[w_rowIndex]++;
        }
        for (q_i = 1; q_i <= w_nRows; q_i++)
        {
            pw_pos[q_i] = pw_pos[q_i - 1] + pw_num[q_i - 1];
        }
        OS_FREE(pw_num);
    }
    return 1;
}
/**
 * @brief get number of columns in the matrix,the value type is double
 * @param[in]    pz_sparseMatrix represent sparse matrix A
 * @param[out]   pw_pos is the number of columns in the matrix
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t findNonZeroColPosDouble(const gnss_sparseDoubleMatrix* pz_sparseMatrix, uint16_t* pw_pos)
{
    uint16_t w_nCols = (pz_sparseMatrix->w_colNum);
    uint32_t q_eleNum = (pz_sparseMatrix->q_eleNum);
    uint32_t q_i = 0;
    int16_t w_colIndex = 0;
    uint16_t* pw_num = NULL;
    for (q_i = 0; q_i < q_eleNum; ++q_i)
    {
        w_colIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_columIndex;
        if (w_colIndex < 0 || w_colIndex >= w_nCols)
        {
            return 0;
        }
    }

    pw_num = (uint16_t*)OS_MALLOC(w_nCols * sizeof(uint16_t));
    if (NULL != pw_num)
    {
        for (q_i = 0; q_i < w_nCols; ++q_i)
        {
            pw_num[q_i] = pw_pos[q_i] = 0;
        }
        pw_pos[w_nCols] = 0;
        for (q_i = 0; q_i < q_eleNum; q_i++)
        {
            w_colIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_columIndex;
            pw_num[w_colIndex]++;
        }
        for (q_i = 1; q_i <= w_nCols; q_i++)
        {
            pw_pos[q_i] = pw_pos[q_i - 1] + pw_num[q_i - 1];
        }
        OS_FREE(pw_num);
    }
    return 1;
}
/**
 * @brief get number of columns in the matrix,the value type is float
 * @param[in]    pz_sparseMatrix represent sparse matrix A
 * @param[out]   pw_pos is the number of columns in the matrix
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t findNonZeroColPosFloat(const gnss_sparseFloatMatrix* pz_sparseMatrix, uint16_t* pw_pos)
{
    uint16_t w_nCols = (pz_sparseMatrix->w_colNum);
    uint32_t q_eleNum = (pz_sparseMatrix->q_eleNum);
    uint32_t q_i = 0;
    int16_t w_colIndex = 0;
    uint16_t* pw_num = NULL;
    for (q_i = 0; q_i < q_eleNum; ++q_i)
    {
        w_colIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_columIndex;
        if (w_colIndex < 0 || w_colIndex >= w_nCols)
        {
            return 0;
        }
    }
    pw_num = (uint16_t*)OS_MALLOC(w_nCols * sizeof(uint16_t));
    if (NULL != pw_num)
    {
        for (q_i = 0; q_i < w_nCols; ++q_i)
        {
            pw_num[q_i] = pw_pos[q_i] = 0;
        }
        pw_pos[w_nCols] = 0;
        for (q_i = 0; q_i < q_eleNum; q_i++)
        {
            w_colIndex = pz_sparseMatrix->pz_tripleEleSet[q_i].w_columIndex;
            pw_num[w_colIndex]++;
        }
        for (q_i = 1; q_i <= w_nCols; q_i++)
        {
            pw_pos[q_i] = pw_pos[q_i - 1] + pw_num[q_i - 1];
        }
        OS_FREE(pw_num);
    }
    return 1;
}
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
    const double* pd_rightMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, double* pd_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nRows = (pz_leftMatrix->w_rowNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_colIndex = 0;
    double d_temp = 0.0;
    double d_deltaValue = 0.0;
    if (w_rightRowNum <= 0 || w_rightColNum <= 0 || (pz_leftMatrix->w_rowNum) <= 0
        || (pz_leftMatrix->w_colNum) <= 0 || w_rightRowNum != (pz_leftMatrix->w_colNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nRows + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nRows + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroRowPosDouble(pz_leftMatrix, pw_pos))
    {

        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_nRows; ++w_i)
    {
        w_startIndex = pw_pos[w_i];
        w_endIndex = pw_pos[w_i + 1];

        if (w_startIndex == w_endIndex)
        {
            continue;
        }
        for (w_j = 0; w_j < w_rightColNum; ++w_j)
        {
            d_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_colIndex = pz_leftMatrix->pz_tripleEleSet[w_k].w_columIndex;
                d_temp += (pz_leftMatrix->pz_tripleEleSet[w_k].d_value * pd_rightMatrix[w_colIndex * w_rightColNum + w_j]);
            }
            pd_result[w_i * w_rightColNum + w_j] = d_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
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
    const float* pf_rightMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, float* pf_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nRows = (pz_leftMatrix->w_rowNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_colIndex = 0;
    float f_temp = 0.0;
    float f_deltaValue = 0.0;
    if (w_rightRowNum <= 0 || w_rightColNum <= 0 || (pz_leftMatrix->w_rowNum) <= 0
        || (pz_leftMatrix->w_colNum) <= 0 || w_rightRowNum != (pz_leftMatrix->w_colNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nRows + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nRows + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroRowPosFloat(pz_leftMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_nRows; ++w_i)
    {
        w_startIndex = pw_pos[w_i];
        w_endIndex = pw_pos[w_i + 1];

        if (w_startIndex == w_endIndex)
        {
            continue;
        }
        for (w_j = 0; w_j < w_rightColNum; ++w_j)
        {
            f_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_colIndex = pz_leftMatrix->pz_tripleEleSet[w_k].w_columIndex;
                f_temp += (pz_leftMatrix->pz_tripleEleSet[w_k].f_value * pf_rightMatrix[w_colIndex * w_rightColNum + w_j]);
            }
            pf_result[w_i * w_rightColNum + w_j] = f_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
	
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
    const double* pd_symMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, double* pd_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nRows = (pz_leftMatrix->w_rowNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_colIndex = 0;
    double d_temp = 0.0;
    double d_deltaValue = 0.0;
    if (w_rightRowNum <= 0 || w_rightColNum <= 0 || (pz_leftMatrix->w_rowNum) <= 0
        || (pz_leftMatrix->w_colNum) <= 0 || w_rightRowNum != (pz_leftMatrix->w_colNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nRows + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nRows + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroRowPosDouble(pz_leftMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_nRows; ++w_i)
    {
        w_startIndex = pw_pos[w_i];
        w_endIndex = pw_pos[w_i + 1];

        if (w_startIndex == w_endIndex)
        {
            continue;
        }
        for (w_j = 0; w_j < w_rightColNum; ++w_j)
        {
            d_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_colIndex = pz_leftMatrix->pz_tripleEleSet[w_k].w_columIndex;
                d_temp += (pz_leftMatrix->pz_tripleEleSet[w_k].d_value * pd_symMatrix[IUTM(w_colIndex, w_j)]);
            }
            pd_result[w_i * w_rightColNum + w_j] = d_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
	
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
    const float* pf_symMatrix, uint16_t w_rightRowNum, uint16_t w_rightColNum, float* pf_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nRows = (pz_leftMatrix->w_rowNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_colIndex = 0;
    float f_temp = 0.0;
    float f_deltaValue = 0.0;
    if (w_rightRowNum <= 0 || w_rightColNum <= 0 || (pz_leftMatrix->w_rowNum) <= 0
        || (pz_leftMatrix->w_colNum) <= 0 || w_rightRowNum != (pz_leftMatrix->w_colNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nRows + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nRows + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroRowPosFloat(pz_leftMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_nRows; ++w_i)
    {
        w_startIndex = pw_pos[w_i];
        w_endIndex = pw_pos[w_i + 1];

        if (w_startIndex == w_endIndex)
        {
            continue;
        }
        for (w_j = 0; w_j < w_rightColNum; ++w_j)
        {
            f_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_colIndex = pz_leftMatrix->pz_tripleEleSet[w_k].w_columIndex;
                f_temp += (pz_leftMatrix->pz_tripleEleSet[w_k].f_value * pf_symMatrix[IUTM(w_colIndex, w_j)]);
            }
            pf_result[w_i * w_rightColNum + w_j] = f_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
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
    uint16_t w_leftColNum, const gnss_sparseDoubleMatrix* pz_rightMatrix, double* pd_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nCols = (pz_rightMatrix->w_colNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_num = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_rowIndex = 0;
    double d_temp = 0.0;
    double d_deltaValue = 0.0;
    if (w_leftRowNum <= 0 || w_leftColNum <= 0 || (pz_rightMatrix->w_rowNum) <= 0 
        || (pz_rightMatrix->w_colNum) <= 0 || w_leftColNum != (pz_rightMatrix->w_rowNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nCols + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nCols + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroColPosDouble(pz_rightMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_leftRowNum; ++w_i)
    {
        w_num = w_i * (pz_rightMatrix->w_colNum);
        for (w_j = 0; w_j < pz_rightMatrix->w_colNum; ++w_j)
        {
            w_startIndex = pw_pos[w_j];
            w_endIndex = pw_pos[w_j + 1];

            d_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_rowIndex = pz_rightMatrix->pz_tripleEleSet[w_k].w_rowIndex;
                d_temp += (pz_rightMatrix->pz_tripleEleSet[w_k].d_value * pd_leftMatrix[w_i * w_leftColNum + w_rowIndex]);
            }
            pd_result[w_num + w_j] = d_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
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
    uint16_t w_leftColNum, const gnss_sparseFloatMatrix* pz_rightMatrix, float* pf_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nCols = (pz_rightMatrix->w_colNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_num = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_rowIndex = 0;
    float f_temp = 0.0;
    float f_deltaValue = 0.0;
    if (w_leftRowNum <= 0 || w_leftColNum <= 0 || (pz_rightMatrix->w_rowNum) <= 0
        || (pz_rightMatrix->w_colNum) <= 0 || w_leftColNum != (pz_rightMatrix->w_rowNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nCols + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nCols + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroColPosFloat(pz_rightMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_leftRowNum; ++w_i)
    {
        w_num = w_i * (pz_rightMatrix->w_colNum);
        for (w_j = 0; w_j < pz_rightMatrix->w_colNum; ++w_j)
        {
            w_startIndex = pw_pos[w_j];
            w_endIndex = pw_pos[w_j + 1];

            f_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_rowIndex = pz_rightMatrix->pz_tripleEleSet[w_k].w_rowIndex;
                f_temp += (pz_rightMatrix->pz_tripleEleSet[w_k].f_value * pf_leftMatrix[w_i * w_leftColNum + w_rowIndex]);
            }
            pf_result[w_num + w_j] = f_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
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
    uint16_t w_leftColNum, const gnss_sparseDoubleMatrix* pz_rightMatrix, double* pd_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nCols = (pz_rightMatrix->w_colNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_rowIndex = 0;
    double d_temp = 0.0;
    double d_deltaValue = 0.0;
    if (w_leftRowNum <= 0 || w_leftColNum <= 0 || (pz_rightMatrix->w_rowNum) <= 0
        || (pz_rightMatrix->w_colNum) <= 0 || w_leftColNum != (pz_rightMatrix->w_rowNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nCols + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nCols + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroColPosDouble(pz_rightMatrix, pw_pos))
    {
        OS_FREE(pw_pos);
        return 0;
    }
    for (w_i = 0; w_i < w_leftRowNum; ++w_i)
    {
        for (w_j = 0; w_j <= w_i; ++w_j)
        {
            w_startIndex = pw_pos[w_j];
            w_endIndex = pw_pos[w_j + 1];

            d_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_rowIndex = pz_rightMatrix->pz_tripleEleSet[w_k].w_rowIndex;
                d_temp += (pz_rightMatrix->pz_tripleEleSet[w_k].d_value * pd_leftMatrix[w_i * w_leftColNum + w_rowIndex]);
            }
            pd_result[IUTM(w_i, w_j)] = d_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}
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
    uint16_t w_leftColNum, const gnss_sparseFloatMatrix* pz_rightMatrix, float* pf_result)
{
    uint16_t* pw_pos = NULL;
    uint16_t w_nCols = (pz_rightMatrix->w_colNum);
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    uint16_t w_startIndex = 0;
    uint16_t w_endIndex = 0;
    int16_t w_rowIndex = 0;
    float f_temp = 0.0;
    float f_deltaValue = 0.0;
    if (w_leftRowNum <= 0 || w_leftColNum <= 0 || (pz_rightMatrix->w_rowNum) <= 0
        || (pz_rightMatrix->w_colNum) <= 0 || w_leftColNum != (pz_rightMatrix->w_rowNum))
    {
        return 0;
    }
    pw_pos = (uint16_t*)OS_MALLOC((w_nCols + 1) * sizeof(uint16_t));
    for (w_i = 0; w_i < w_nCols + 1; ++w_i)
    {
        pw_pos[w_i] = 0;
    }
    if (0 == findNonZeroColPosFloat(pz_rightMatrix, pw_pos))
    {
      OS_FREE(pw_pos);
      return 0;
    }
    for (w_i = 0; w_i < w_leftRowNum; ++w_i)
    {
        for (w_j = 0; w_j <= w_i; ++w_j)
        {
            w_startIndex = pw_pos[w_j];
            w_endIndex = pw_pos[w_j + 1];

            f_temp = 0.0;
            for (w_k = w_startIndex; w_k < w_endIndex; ++w_k)
            {
                w_rowIndex = pz_rightMatrix->pz_tripleEleSet[w_k].w_rowIndex;
                f_temp += (pz_rightMatrix->pz_tripleEleSet[w_k].f_value * pf_leftMatrix[w_i * w_leftColNum + w_rowIndex]);
            }
            pf_result[IUTM(w_i, w_j)] = f_temp;
        }
    }
    OS_FREE(pw_pos);
    return 1;
}