/**@file        fusion_math.h
 * @brief		fusion math header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_MATH_H__
#define __FUSION_MATH_H__

#include <stdint.h>
#include <math.h>

#define MAXMATCNT (30)

void math_mateye(float* pf_a, uint32_t u_dim);
void math_cross_product(float pf_a[3], float pf_b[3], float pf_c[3]);
void math_skewsym(float pf_vct[3], float pf_mat[3][3]);
void math_matadd(float pf_a[], float pf_b[], uint32_t u_row, uint32_t u_col, float pf_c[]);
void math_matsub(float pf_a[], float pf_b[], uint32_t u_row, uint32_t u_col, float pf_c[]);
void math_mattrns(float pf_a[], uint32_t u_row, uint32_t u_col, float pf_b[]);
void math_matmul(float pf_a[], float pf_b[], uint32_t u_arow, uint32_t u_acol, uint32_t u_bcol, float pf_c[]);
void math_matmul_const(float pf_a[], float f_b, uint32_t u_row, uint32_t u_col, float pf_c[]);
void math_square_transpose(float f_m[], uint32_t u_rows);
void math_matcopy(float f_a[], float f_b[], uint32_t u_row, uint32_t u_col);
int8_t math_choleskydc(float* pf_m, uint32_t u_cols);
int8_t math_matinv(float* pf_m, uint32_t u_dim, float* pf_det);
void math_matextract(float f_a[], uint32_t pn, float f_b[], uint32_t u_cm, uint32_t u_cn, uint32_t u_startm, uint32_t u_endn);
uint8_t math_mat_calinverse(float* pf_matinv, uint32_t u_dim);
float math_calculate_mean(float* pf_array, int32_t q_size);
void math_calculate_std(float* pf_buf, uint32_t q_len, float* pf_mean, float* pf_std);
void math_calculate_maxmindiff(float* pf_buf, uint32_t q_len, float* pf_diff);
void math_matrix_transpose(float f_a[], uint32_t u_row, uint32_t u_col, float f_b[]);
void math_bised_ave_std(float* pf_average, float* pf_std, float f_income, uint32_t u_count);
int8_t double_equal(double a, double b);
int8_t float_equal(float a, float b);

#endif // !__FUSION_MATH_H__
