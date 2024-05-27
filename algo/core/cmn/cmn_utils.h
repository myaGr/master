/**@file        cmn_utils.h
 * @brief       Common useful reenterable funtions
 * @details     
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

#ifndef __CMN_UTILS_H__
#define __CMN_UTILS_H__

#include "cmn_def.h"

BEGIN_DECL

#define M_SET_BIT(v, b)     ((v) |= ((uint64_t)1<<(b)))
#define M_CLEAR_BIT(v, b)   ((v) &= ~((uint64_t)1<<(b)))
#define M_GET_BIT(v, b)     ((v) & ((uint64_t)1<<(b)))
#define M_IS_SET_BIT(v, b)  (((uint64_t)v) & ((uint64_t)1<<b))
#define M_SET_MASK(v, m)    ((v) |= (m))
#define M_CLEAR_MASK(v, m)  ((v) &= ~(m))
#define M_IS_SET_MASK(v, m) (((v) & (m)) == (m))
#define M_ARRAY_SIZE(v)     (sizeof(v)/sizeof(v[0]))
#define M_MIN(x, y)         ((x) < (y) ? (x) : (y))
#define M_MAX(x, y)         ((x) > (y) ? (x) : (y))

/**
 * @brief Get software version information
 * @param[in] version_str - version buffer
 * @return n - version string length
 */
uint32_t get_version(char* version_str);

/* get fields (little-endian) ------------------------------------------------*/
unsigned  char getU1(unsigned char* p);
          char getI1(unsigned char* p);
unsigned short getU2(unsigned char* p);
         short getI2(unsigned char* p);
unsigned   int getU4(unsigned char* p);
           int getI4(unsigned char* p);
         float getR4(unsigned char* p);
        double getR8(unsigned char* p);

/* set fields (little-endian) ------------------------------------------------*/
       uint8_t setU1(unsigned char* p, unsigned char  u);
       uint8_t setU2(unsigned char* p, unsigned short u);
       uint8_t setU4(unsigned char* p, unsigned int   u);
       uint8_t setI1(unsigned char* p, signed char    i);
       uint8_t setI2(unsigned char* p, short          i);
       uint8_t setI4(unsigned char* p, int            i);
       uint8_t setR4(unsigned char* p, float          r);
       uint8_t setR8(unsigned char* p, double         r);

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : uint8_t *buff    I   byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
uint32_t loc_getbitu(const uint8_t* buff, int pos, int len);

int32_t loc_getbits(const uint8_t* buff, int pos, int len);

/* get signed 38bit field ----------------------------------------------------*/
double loc_getbits_38(const unsigned char* buff, int pos);

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : uint8_t *buff IO byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
*          [u]int32_t data  I   unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
void loc_setbitu(uint8_t* buff, int pos, int len, uint32_t data);

void loc_setbits(uint8_t* buff, int pos, int len, int32_t data);

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
uint32_t loc_crc24q(const uint8_t* buff, int len);

/**
 * @brief Compute 2 Byte checksum from buffer
 * @param[in] data :
 * @param[in] len :
 * @param[in] ck0 :
 * @param[in] ck1 :
 * @return: None
 */
void loc_checksum(uint8_t* data, uint32_t len, uint8_t* ck0, uint8_t* ck1);

/**
 * @brief Check Any pointers are valid
 * @param[in] l_NumPtr - pointer number
 * @return TRUE - pointers has NULL
 */
uint8_t any_Ptrs_Null(int32_t l_NumPtr, ...);

typedef struct {
  uint32_t row; /** row    number */
  uint32_t col; /** column number */
  BOOL     b_heap; /** flag to indicate data is in heap*/
  double* data;
} matrix_t;

/** Triangular matrix
  data store the non-zero value.
       dim
  | x ...... x |
  | 0  x ... x |
  | 0 ... x  x |
  | 0 ...... x | */
typedef struct {
  uint32_t dim; /** dimension number */
  double* data;
} matrix_tri_t;

/**
 * @brief Create a new matrix by dynamic memory allocate
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new(uint32_t row, uint32_t col);

/**
 * @brief Get a element address in a  matrix
 * @param[in] m
 * @param[in] x
 * @param[in] y
 * @return pointer to field
 */
double* matrix_get(const matrix_t* m, uint32_t x, uint32_t y);

/** Get a unit at matrix(x, y) */
#define MAT(m, x, y)     (*matrix_get(m, x, y))

/**
 * @brief Create a new matrix from input array
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] array - input array
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_array(uint32_t row, uint32_t col, const double* array);

/**
 * @brief Create a new matrix from input array
 * @param[in] row - matrix row number
 * @param[in] list - input list
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_list(uint32_t row, const double* array);

/**
 * @brief Create a new matrix from input buffer
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] buffer - input buffer
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_buffer(uint32_t row, uint32_t col, double* buffer);

/**
 * @brief Create a new matrix from input buffer, Const defines
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] buffer - input buffer
 * @return matrix_t - matrix_t pointer
 */
const matrix_t* matrix_new_from_buffer_const(uint32_t row, uint32_t col, double* buffer);

/**
 * @brief Clean a matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_clean(matrix_t* mat);

/**
 * @brief Free a matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_free(matrix_t** mat);

/**
 * @brief Copy matrix to a new matrix
          The new matrix has own memory
 * @param[in] mat
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_clone(const matrix_t* mat);

/**
 * @brief Copy matrix to a exist matrix
          The new matrix has own memory
 * @param[in] dst
 * @param[in] src
 * @return TRUE - Success
           FALSE - Fail
 */
BOOL matrix_cpy(matrix_t* dst, matrix_t* src);

/**
 * @brief Copy matrix to a new square matrix
 * @param[in] dim - square matrix dimension
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_ones(uint32_t dim);

/**
 * @brief Transpose matrix
 * @param[in/out] mat
 * @return None
 */
void matrix_transpose(matrix_t* mat);

/**
 * @brief Matrix A + Matrix B
 * @param[in] ma
 * @param[in] mb
 * @return None
 */
void matrix_add(matrix_t* ma, matrix_t* mb);

/**
 * @brief Matrix A - Matrix B
 * @param[in] ma
 * @param[in] mb
 * @return None
 */
void matrix_minus(matrix_t* ma, matrix_t* mb);

/**
 * @brief Matrix scale*A+add
 * @param[in] mat
 * @param[in] add
 * @param[in] scale
 * @return None
 */
void matrix_scale(matrix_t* mat, double scale, double add);

/**
 * @brief Part of Matrix scale*A+add
 * @param[in] mat
 * @param[in] begin_row
 * @param[in] begin_col
 * @param[in] end_row
 * @param[in] end_col
 * @param[in] scale
 * @param[in] add
 * @return None
 */
void matrix_part_scale(matrix_t* mat, int32_t begin_row, int32_t begin_col, 
  int32_t end_row, int32_t end_col, double scale, double add);

/**
 * @brief Matrix index value check
 * @param[in] idx
 * @param[in] idx_min
 * @param[in] idx_max
 * @return None
 */
int32_t matrix_idx_check(int32_t idx, int32_t idx_min, int32_t idx_max);

/**
 * @brief Matrix mat_a * mat_b = mat_o
 * @param[in] mat_a
 * @param[in] mat_b
 * @param[out] mat_o - output matrix
 * @return None
 */
void matrix_mul(const matrix_t* mat_a, const matrix_t* mat_b, matrix_t* mat_o);

/**
 * @brief Compute the 2-norm of an row vector in a matrix
          norm = (v_1 * v_1 + ... + v_n * v_n)^(1/2)
          | *   *  ...  *  |
          | *  v_1 ... v_n |
          | *   *  ...  *  |
          | *   *  ...  *  |
 * @param[in] mat
 * @param[in] row
 * @param[in] start_col
 * @param[in] end_col
 * @return 2-norm value
 */
double matrix_row_vector_norm(matrix_t* mat, int32_t row, int32_t start_col, int32_t end_col);

/**
 * @brief Compute the 2-norm of an col vector in a matrix
          norm = (v_1 * v_1 + ... + v_n * v_n)^(1/2)
          | *   *   *  |
          | *  v_1  *  |
          | *   .   *  |
          | *  v_n  *  |
          | *   *   *  |
 * @param[in] mat
 * @param[in] row
 * @param[in] start_col
 * @param[in] end_col
 * @return 2-norm value
 */
double matrix_col_vector_norm(matrix_t* mat, int32_t col, int32_t start_row, int32_t end_row);

/**
 * @brief Compute dot of two row vectors in matrix
 * @param[in] a - vectot a
 * @param[in] b - vectot b
 * @return matrix dot
 */
double matrix_row_vector_dot(matrix_t* a, int32_t a_row, matrix_t* b, int32_t b_row);

/**
 * @brief Compute dot of two col vectors in matrix
 * @param[in] a - vectot a
 * @param[in] b - vectot b
 * @return matrix dot
 */
double matrix_col_vector_dot(matrix_t* a, int32_t a_col, matrix_t* b, int32_t b_col);

/**
 * @brief Copy partial matrix into dst from src
 * @param[in]
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL matrix_partial_copy(matrix_t* dst, uint32_t dst_row_idx, uint32_t dst_col_idx,
  matrix_t* src, uint32_t src_row_idx, uint32_t src_col_idx,
  uint32_t row_count, uint32_t col_count);

/**
 * @brief LU Decompose
          m = L * U, L is lower triangular matrix
                     U is upper triangular matrix
 * @param[in] mat
 * @param[out] l
 * @param[out] l
 * @return None
 */
void matrix_lu_decompose(matrix_t* mat,
  matrix_t* l, matrix_t* u);

/**
 * @brief QR Decompose using Gram Schmidt algorothm
          m = Q * R, Q is a orthogonal matrix Q^-1 = Q^t
                     R is an upper/right triangular matrix
          m^-1 = (Q * R)^-1
               = R^-1*Q^-1
               = R^-1*Q^t
 * @param[in]  A
 * @param[out] q
 * @param[out] r
 * @return TRUE - success
 */
BOOL matrix_qr_decompose(matrix_t* A, matrix_t* Q, matrix_t* R);

/**
 * @brief QR Decompose using HouseHolder algorothm
          A = Q * R, Q is a orthogonal matrix Q^-1 = Q^t
                     R is an upper/right triangular matrix
 * @param[in]  A(n*m)
 * @param[out] Qt is the transpose matrix of Q(n*n)
 * @param[out] R(n*m) is an upper/right triangular matrix
 * @return TRUE - success
 */
BOOL matrix_qrParityDecompose(matrix_t* A, matrix_t* Qt, matrix_t* R);
/**
 * @brief Matrix Inverse
 * @param[in] mat
 * @param[out] mat_inv
 * @return BOOL
 */
BOOL matrix_inverse(const matrix_t* mat, matrix_t* mat_inv);

/**
 * @brief Right Up Matrix Inverse.
          Right Up Matrix inverse is much simpler than general matrix
 * @param[in] mat
 * @param[out] mat_inv
 * @return None
 */
BOOL matrix_rightup_inverse(matrix_t* mat, matrix_t* mat_inv);

/**
 * @brief Matrix Print
 * @param[in] mat
 * @return None
 */
void matrix_print(matrix_t* mat);

/**
 * @brief Create a new upper triangular matrix by dynamic memory allocate
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @return matrix_t - matrix_t pointer
 */
matrix_tri_t* matrix_tri_new(uint32_t dim);

/**
 * @brief Get a element address in a upper triangular matrix
 * @param[in] m - upper triangular matrix
 * @param[in] x
 * @param[in] y
 * @return pointer to field
 */
double* matrix_tri_get(matrix_tri_t* m, uint32_t x, uint32_t y);

/** Get a unit at upper triangular matrix (x, y) */
#define MAT_TRI(m, x, y) (*matrix_tri_get(m, x, y))

/**
 * @brief Create a new upper triangular matrix from input array
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] array - input array
 * @return matrix_t - matrix_t pointer
 */
matrix_tri_t* matrix_tri_new_from_array(uint32_t dim, double* array[]);

/**
 * @brief Free a upper triangular matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_tri_free(matrix_tri_t* mat);

/**
 * @brief Convert a upper triangular matrix to square matrix
 * @param[in] mat_uptri
 * @param[out] mat - square matrix
 * @return None
 */
BOOL matrix_cvt_tri2square(matrix_tri_t* mat_uptri, matrix_t* mat);

/**
 * @brief Convert a square matrix to a upper triangular matrix
 * @param[in] mat - square matrix
 * @param[out] mat_uptri
 * @return TRUE - Success
           FALSE - Fail
 */
BOOL matrix_cvt_square2tri(matrix_t* mat, matrix_tri_t* mat_uptri);

/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : matrix_t *H        I   (weighted) design matrix (n x m)
*          matrix_t *y        I   (weighted) measurements (n x 1)
*          matrix_t *x        O   estmated parameters (m x 1)
*          matrix_t *Q        O   esimated parameters covariance matrix (m x m)
* return : status (1:ok,0:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*-----------------------------------------------------------------------------*/
BOOL lsqMat(const matrix_t* H, const matrix_t* y, matrix_t* x, matrix_t* Q);

/**
 * @brief Sort general function based on Insert Sort Algorithm
 * @param[in/out] data
 * @param[in]   step - each data step
 * @param[in]   count - data count
 * @param[in]   compare - compare funtion
 * @return      None
 */
void loc_sort(uint8_t* data, uint8_t step, uint8_t count, BOOL(*compare)(void*, void*));

/**
 * @brief computes int8_t type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
int32_t vector_u8_sum(const uint8_t* u_vals, int32_t q_nums);

/**
 * @brief computes int32_t type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
int32_t vector_q32_sum(const int32_t* q_vals, int32_t q_nums);

/**
 * @brief Max value in double type array
 * @param[in] pd_data
 * @param[in] q_num
 * @return max value in double type array
 */
double vector_dbl_max(const double* pd_data, uint32_t q_num);

/**
 * @brief Min value in double type array
 * @param[in] pd_data
 * @param[in] q_num
 * @return min value in double type array
 */
double vector_dbl_min(const double* pd_data, uint32_t q_num);

/**
 * @brief computes float type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
float vector_flt_sum(const float* d_vals, int32_t q_nums);

/**
 * @brief computes double type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
double vector_dbl_sum(const double* d_vals, int32_t q_nums);

/**
 * @brief computes the 2-norm of an arbitrary length vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      the 2-norm of the vector
 */
double vector_norm(const double* d_vals, uint32_t q_nums);

/**
 * @brief Free all matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_free_batch(int32_t l_NumPtr, ...);

/**
 * @brief Safe free pointer created by malloc
 * @param[in,out] ptr
 */
void safe_free_p(void *ptr);

/**
 * @brief Get sum of '1' in binary format marklist for specific LSB
 * @param[in] t_src marklist
 * @param[in] u_lsbwide LSB length
 * @return uint8_t count
 */
uint8_t sum_of_bit(uint64_t t_src, uint8_t u_lsbwide);

END_DECL

#endif
