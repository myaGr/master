/**@file        cmn_utils.c
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

#include <stdarg.h>
#include "mw_alloc.h"
#include "cmn_utils.h"

/**
 * @brief Get software version information
 * @param[in] version_str - version buffer
 * @return n - version string length
 */
uint32_t get_version(char* version_str)
{
  uint32_t n = 0;
#ifdef GIT_TAG
  n += sprintf(version_str + n, "Version:%s", GIT_TAG);
#endif
#ifdef GIT_BRANCH
  n += sprintf(version_str + n, " branch:%s", GIT_BRANCH);
#endif
#ifdef GIT_COMMIT_ID
  n += sprintf(version_str + n, " commit:%s", GIT_COMMIT_ID);
#endif
  n += sprintf(version_str + n, " Build Time %s-%s", __TIME__, __DATE__);
  return n;
}

static const uint32_t tbl_CRC24Q[] = {
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

/* get fields (little-endian) ------------------------------------------------*/
unsigned  char getU1(unsigned char* p) { unsigned char  u; memcpy(&u, p, 1); return u; }
          char getI1(unsigned char* p) {          char  u; memcpy(&u, p, 1); return u; }
unsigned short getU2(unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; }
         short getI2(unsigned char* p) { short          i; memcpy(&i, p, 2); return i; }
unsigned   int getU4(unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }
           int getI4(unsigned char* p) { int            u; memcpy(&u, p, 4); return u; }
         float getR4(unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }
        double getR8(unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }

/* set fields (little-endian) ------------------------------------------------*/
       uint8_t setU1(unsigned char* p, unsigned char  u) { *p = u;                 return 1;}
       uint8_t setU2(unsigned char* p, unsigned short u) { memcpy(p, &u, 2);       return 2;}
       uint8_t setU4(unsigned char* p, unsigned int   u) { memcpy(p, &u, 4);       return 4;}
       uint8_t setI1(unsigned char* p, signed char    i) { *p = (unsigned char)i;  return 1;}
       uint8_t setI2(unsigned char* p, short          i) { memcpy(p, &i, 2);       return 2;}
       uint8_t setI4(unsigned char* p, int            i) { memcpy(p, &i, 4);       return 4;}
       uint8_t setR4(unsigned char* p, float          r) { memcpy(p, &r, 4);       return 4;}
       uint8_t setR8(unsigned char* p, double         r) { memcpy(p, &r, 8);       return 8;}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : uint8_t *buff    I   byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
uint32_t loc_getbitu(const uint8_t* buff, int pos, int len)
{
  uint32_t bits = 0;
  int i;
  for (i = pos; i < pos + len; i++) bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  return bits;
}

int32_t loc_getbits(const uint8_t* buff, int pos, int len)
{
  uint32_t bits = loc_getbitu(buff, pos, len);
  if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) return (int32_t)bits;
  return (int32_t)(bits | (~0u << len)); /* extend sign */
}

/* get signed 38bit field ----------------------------------------------------*/
double loc_getbits_38(const unsigned char* buff, int pos)
{
  return (double)loc_getbits(buff, pos, 32) * 64.0 + loc_getbitu(buff, pos + 32, 6);
}

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : uint8_t *buff IO byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
*          [u]int32_t data  I   unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
void loc_setbitu(uint8_t* buff, int pos, int len, uint32_t data)
{
  uint32_t mask = 1u << (len - 1);
  int i;
  if (len <= 0 || 32 < len) {
    return;
  }
  for (i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask) {
      buff[i / 8] |= 1u << (7 - i % 8);
    }
    else {
      buff[i / 8] &= ~(1u << (7 - i % 8));
    }
  }
}

void loc_setbits(uint8_t* buff, int pos, int len, int32_t data)
{
  if (data < 0) {
    data |= 1 << (len - 1);
  }
  else {
    data &= ~(1 << (len - 1)); /* set sign bit */
  }

  loc_setbitu(buff, pos, len, (uint32_t)data);
}

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
uint32_t loc_crc24q(const uint8_t* buff, int len)
{
  uint32_t crc = 0;
  int i;

  for (i = 0; i < len; i++)
  {
    crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
  }

  return crc;
}

/**
 * @brief Compute 2 Byte checksum from buffer
 * @param[in] data :
 * @param[in] len :
 * @return: None
 */
void loc_checksum(uint8_t* data, uint32_t len, uint8_t* ck0, uint8_t* ck1)
{
  uint8_t CK[2] = { *ck0, *ck1 };

  for (uint32_t i = 0; i < len; i++)
  {
    CK[0] = CK[0] + data[i];
    CK[1] = CK[1] + CK[0];
  }

  *ck0 = CK[0];
  *ck1 = CK[1];
  return;
}

/**
 * @brief Check Any pointer is valid
 * @param[in] l_NumPtr - pointer number
 * @return TRUE - pointers has NULL
 */
uint8_t any_Ptrs_Null(int32_t l_NumPtr, ...)
{
  int32_t l_PtrIdx = 0;
  uint8_t v_NullPtrChkFailed = FALSE;

  va_list z_Args;

  va_start(z_Args, l_NumPtr);

  for (l_PtrIdx = 0; l_PtrIdx < l_NumPtr; l_PtrIdx++)
  {
    if (NULL == va_arg(z_Args, void*))
    {
      v_NullPtrChkFailed = TRUE;
      break;
    }
  }
  va_end(z_Args);

  return v_NullPtrChkFailed;
}

/**
 * @brief Create a new matrix by dynamic memory allocate
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new(uint32_t row, uint32_t col)
{
  matrix_t* mat = (matrix_t*)OS_MALLOC_FAST(sizeof(matrix_t));
  if (NULL == mat)
  {
    return NULL;
  }

  mat->row = row;
  mat->col = col;
  mat->data = (void*)OS_MALLOC_FAST(row * col * sizeof(double));
  mat->b_heap = TRUE;
  if (NULL == mat->data)
  {
    OS_FREE(mat);
    mat = NULL;
    return NULL;
  }
  memset(mat->data, 0, row * col * sizeof(double));

  return mat;
}

/**
 * @brief Get a element address in a  matrix
 * @param[in] m
 * @param[in] x
 * @param[in] y
 * @return pointer to field
 */
double* matrix_get(const matrix_t* m, uint32_t x, uint32_t y)
{
  if (!(x < (m->row) && y < (m->col)))
  {
    return &(m->data[0]);
  }

  uint32_t idx = (x * m->col) + y;

  return &(m->data[idx]);
}

/**
 * @brief Create a new matrix from input array
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] array - input array
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_array(uint32_t row, uint32_t col, const double* array)
{
  uint32_t i = 0;
  uint32_t j = 0;
  matrix_t* mat = matrix_new(row, col);
  if (NULL == mat)
  {
    return NULL;
  }

  for (i = 0; i < mat->row; i++)
  {
    for (j = 0; j < mat->col; j++)
    {
      MAT(mat, i, j) = *(array + i * col + j);
    }
  }

  return mat;
}

/**
 * @brief Create a new matrix from input array
 * @param[in] row - matrix row number
 * @param[in] list - input list
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_list(uint32_t row, const double* array)
{
  uint32_t i = 0;
  uint32_t j = 0;
  matrix_t* mat = matrix_new(row, 1);
  if (NULL == mat)
  {
    return NULL;
  }

  for (i = 0; i < mat->row; i++)
  {
    for (j = 0; j < mat->col; j++)
    {
      MAT(mat, i, j) = array[j + i * mat->col];
    }
  }

  return mat;
}

/**
 * @brief Create a new matrix from input buffer
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] buffer - input buffer
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_new_from_buffer(uint32_t row, uint32_t col, double* buffer)
{
  if (NULL == buffer)
  {
    return NULL;
  }

  matrix_t* mat = (matrix_t*)OS_MALLOC(sizeof(matrix_t));
  if (NULL == mat)
  {
    return NULL;
  }

  mat->row = row;
  mat->col = col;
  mat->data = buffer;
  mat->b_heap = FALSE;
  return mat;
}

/**
 * @brief Create a new matrix from input buffer
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] buffer - input buffer
 * @return matrix_t - matrix_t pointer
 */
const matrix_t* matrix_new_from_buffer_const(uint32_t row, uint32_t col, double* buffer)
{
  if (NULL == buffer)
  {
    return NULL;
  }

  matrix_t* mat = (matrix_t*)OS_MALLOC(sizeof(matrix_t));
  if (NULL == mat)
  {
    return NULL;
  }

  mat->row = row;
  mat->col = col;
  mat->data = buffer;
  mat->b_heap = FALSE;
  return (const matrix_t*)mat;
}

/**
 * @brief Clean a matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_clean(matrix_t* mat)
{
  if (NULL == mat)
  {
    return;
  }

  memset(mat->data, 0, mat->col * mat->row * sizeof(double));
}

/**
 * @brief Free a matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_free(matrix_t** mat)
{
  if ((NULL == mat) || (NULL == *mat))
  {
    return;
  }

  if ((TRUE == (*mat)->b_heap) && ((*mat)->data))
  {
    OS_FREE((*mat)->data);
    (*mat)->data = NULL;
  }

  OS_FREE((*mat));
  (*mat) = NULL;
}

/**
 * @brief Clone matrix to a new matrix
          The new matrix has own memory
 * @param[in] mat
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_clone(const matrix_t* mat)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(2, mat, mat->data))
  {
    return NULL;
  }

  matrix_t* mat_new = matrix_new(mat->row, mat->col);
  if (NULL == mat_new)
  {
    return NULL;
  }

  for (i = 0; i < mat->row; i++)
  {
    for (j = 0; j < mat->col; j++)
    {
      MAT(mat_new, i, j) = MAT(mat, i, j);
    }
  }
  return mat_new;
}

/**
 * @brief Copy matrix to a exist matrix
          The new matrix has own memory
 * @param[in] dst
 * @param[in] src
 * @return TRUE - Success
           FALSE - Fail
 */
BOOL matrix_cpy(matrix_t* dst, matrix_t* src)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(4, dst, dst->data, src, src->data))
  {
    return FALSE;
  }

  for (i = 0; i < src->row; i++)
  {
    for (j = 0; j < src->col; j++)
    {
      MAT(dst, i, j) = MAT(src, i, j);
    }
  }

  return TRUE;
}

/**
 * @brief Copy matrix to a new square matrix
 * @param[in] dim - square matrix dimension
 * @return matrix_t - matrix_t pointer
 */
matrix_t* matrix_ones(uint32_t dim)
{
  matrix_t* m = matrix_new(dim, dim);
  if (NULL == m)
  {
    return NULL;
  }

  for (uint32_t i = 0; i < dim; i++)
  {
    MAT(m, i, i) = 1.0;
  }

  return m;
}

/**
 * @brief Transpose matrix
 * @param[in/out] mat
 * @return None
 */
void matrix_transpose(matrix_t* mat)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(2, mat, mat->data))
  {
    return;
  }

  matrix_t* m_copy = matrix_clone(mat);
  mat->col = m_copy->row;
  mat->row = m_copy->col;

  for (i = 0; i < m_copy->row; i++)
  {
    for (j = 0; j < m_copy->col; j++)
    {
      MAT(mat, j, i) = MAT(m_copy, i, j);
    }
  }

  matrix_free(&m_copy);
  return;
}

/**
 * @brief Matrix A + Matrix B
 * @param[in] ma
 * @param[in] mb
 * @return None
 */
void matrix_add(matrix_t* ma, matrix_t* mb)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(4, ma, ma->data, mb, mb->data))
  {
    return;
  }

  for (i = 0; i < ma->row; i++)
  {
    for (j = 0; j < ma->col; j++)
    {
      MAT(ma, i, j) += MAT(mb, i, j);
    }
  }

  return;
}

/**
 * @brief Matrix A - Matrix B
 * @param[in] ma
 * @param[in] mb
 * @return None
 */
void matrix_minus(matrix_t* ma, matrix_t* mb)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(4, ma, ma->data, mb, mb->data))
  {
    return;
  }

  for (i = 0; i < ma->row; i++)
  {
    for (j = 0; j < ma->col; j++)
    {
      MAT(ma, i, j) -= MAT(mb, i, j);
    }
  }

  return;
}

/**
 * @brief Matrix scale*A+add
 * @param[in] mat
 * @param[in] add
 * @param[in] scale
 * @return None
 */
void matrix_scale(matrix_t* mat, double scale, double add)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(2, mat, mat->data))
  {
    return;
  }

  for (i = 0; i < mat->row; i++)
  {
    for (j = 0; j < mat->col; j++)
    {
      MAT(mat, i, j) = MAT(mat, i, j) * scale + add;
    }
  }

  return;
}

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
  int32_t end_row, int32_t end_col, double scale, double add)
{
  if (any_Ptrs_Null(2, mat, mat->data))
  {
    return;
  }
  
  begin_row = matrix_idx_check(begin_row, 0, mat->row);
  begin_col = matrix_idx_check(begin_col, 0, mat->col);
  end_row   = matrix_idx_check(end_row,   0, mat->row);
  end_col   = matrix_idx_check(end_col,   0, mat->col);

  for (int i = begin_row; i < end_row; i++)
  {
    for (int j = begin_col; j < end_col; j++)
    {
      MAT(mat, i, j) = MAT(mat, i, j) * scale + add;
    }
  }

  return;
}

/**
 * @brief Matrix index value check
 * @param[in] idx
 * @param[in] idx_min
 * @param[in] idx_max
 * @return None
 */
int32_t matrix_idx_check(int32_t idx, int32_t idx_min, int32_t idx_max)
{
  int32_t k = idx;
  if (idx < idx_min)
  {
    k = idx_min;
  }

  if (idx > idx_max)
  {
    k = idx_max;
  }
  return k;
}

/**
 * @brief Matrix mat_a * mat_b = mat_o
 * @param[in] mat_a
 * @param[in] mat_b
 * @param[out] mat_o - output matrix
 * @return None
 */
void matrix_mul(const matrix_t* mat_a, const matrix_t* mat_b, matrix_t* mat_o)
{
  if (any_Ptrs_Null(6, mat_a, mat_a->data,
    mat_b, mat_b->data,
    mat_o, mat_o->data))
  {
    return;
  }

  if (!((mat_a->row == mat_o->row) &&
    (mat_b->col == mat_o->col) &&
    (mat_a->col == mat_b->row)))
  {
    return;
  }

  uint32_t row = mat_o->row;
  uint32_t col = mat_o->col;
  uint32_t count = mat_a->col;

  for (uint32_t i = 0; i < row; i++)
  {
    for (uint32_t j = 0; j < col; j++)
    {
      MAT(mat_o, i, j) = 0.0;
      for (uint32_t k = 0; k < count; k++)
      {
        MAT(mat_o, i, j) += MAT(mat_a, i, k) * MAT(mat_b, k, j);
      }
    }
  }

  return;
}

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
double matrix_row_vector_norm(matrix_t* mat, int32_t row, int32_t start_col, int32_t end_col)
{
  double sum = 0.0;

  start_col = matrix_idx_check(start_col, 0, mat->col);
  end_col = matrix_idx_check(end_col, 0, mat->col);

  for (int32_t i = start_col; i < end_col; i++)
  {
    sum += mat->data[row * mat->col + i] * mat->data[row * mat->col + i];
  }
  return sqrt(sum);
}

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
double matrix_col_vector_norm(matrix_t* mat, int32_t col, int32_t start_row, int32_t end_row)
{
  double sum = 0.0;

  start_row = matrix_idx_check(start_row, 0, mat->row);
  end_row = matrix_idx_check(end_row, 0, mat->row);

  for (int32_t i = start_row; i < end_row; i++)
  {
    sum += mat->data[i * mat->col + col] * mat->data[i * mat->col + col];
  }
  return sqrt(sum);
}

/**
 * @brief Compute dot of two row vectors in matrix
 * @param[in] a - vectot a
 * @param[in] b - vectot b
 * @return matrix dot
 */
double matrix_row_vector_dot(matrix_t* a, int32_t a_row, matrix_t* b, int32_t b_row)
{
  uint32_t i = 0;
  double sum = 0;

  if (a->col != b->col)
  {
    return 0.0;
  }

  for (i = 0; i < a->col; i++)
  {
    sum = MAT(a, i, a_row) * MAT(b, i, b_row);
  }

  return sum;
}

/**
 * @brief Compute dot of two col vectors in matrix
 * @param[in] a - vectot a
 * @param[in] b - vectot b
 * @return matrix dot
 */
double matrix_col_vector_dot(matrix_t* a, int32_t a_col, matrix_t* b, int32_t b_col)
{
  uint32_t i = 0;
  double sum = 0;

  if (a->row != b->row)
  {
    return 0.0;
  }

  for (i = 0; i < a->row; i++)
  {
    sum += MAT(a, i, a_col) * MAT(b, i, b_col);
  }

  return sum;
}

/**
 * @brief Copy partial matrix into dst from src
 * @param[in]
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL matrix_partial_copy(matrix_t* dst, uint32_t dst_row_idx, uint32_t dst_col_idx,
  matrix_t* src, uint32_t src_row_idx, uint32_t src_col_idx,
  uint32_t row_count, uint32_t col_count)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if ((NULL == src) || (NULL == dst))
  {
    return FALSE;
  }

  if ((src_row_idx > src->row) ||
    ((src_row_idx + row_count) > src->row) ||
    (src_col_idx > src->col) ||
    ((src_col_idx + col_count) > src->col) ||
    (dst_row_idx > dst->row) ||
    ((dst_row_idx + row_count) > dst->row) ||
    (dst_col_idx > dst->col) ||
    ((dst_col_idx + col_count) > dst->col))
  {
    return FALSE;
  }

  for (i = 0; i < row_count; i++)
  {
    for (j = 0; j < col_count; j++)
    {
      dst->data[(i + dst_row_idx) * dst->col + j + dst_col_idx] =
        src->data[(i + src_row_idx) * src->col + j + src_col_idx];
    }
  }

  return TRUE;
}

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
  matrix_t* l, matrix_t* u)
{
  // TBD
  return;
}

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
BOOL matrix_qr_decompose(matrix_t* A, matrix_t* Q, matrix_t* R)
{
  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t k = 0;
  uint32_t row = A->row;
  uint32_t col = A->col;

  MAT(R, 0, 0) = matrix_col_vector_norm(A, 0, 0, A->row);
  if (DBL_EQU(MAT(R, 0, 0), DBL_EPSILON))
  {
    return FALSE;
  }
  else
  {
    matrix_partial_copy(Q, 0, 0, A, 0, 0, A->row, 1);
    matrix_part_scale(Q, 0, 0, Q->row, 1, 1 / MAT(R, 0, 0), 0.0);
  }

  for (i = 1; i < A->col; i++)
  {
    matrix_partial_copy(Q, 0, i, A, 0, i, A->row, 1);
    for (j = 0; j < i; j++)
    {
      MAT(R, j, i) = matrix_col_vector_dot(Q, j, Q, i);
      for (k = 0; k < Q->row; k++)
      {
        MAT(Q, k, i) = MAT(Q, k, i) - MAT(R, j, i) * MAT(Q, k, j);
      }
    }
    MAT(R, i, i) = matrix_col_vector_norm(Q, i, 0, A->row);
    if (!(DBL_EQU(MAT(R, i, i), DBL_EPSILON)))
    {
      for (k = 0; k < Q->row; k++)
      {
        MAT(Q, k, i) = MAT(Q, k, i) / MAT(R, i, i);
      }
    }
  }

  return TRUE;
}

/**
 * @brief QR Decompose using HouseHolder algorothm
          A = Q * R, Q is a orthogonal matrix Q^-1 = Q^t
                     R is an upper/right triangular matrix
 * @param[in]  A(n*m)
 * @param[out] Qt is the transpose matrix of Q(n*n)
 * @param[out] R(n*m) is an upper/right triangular matrix
 * @return TRUE - success
 */
BOOL matrix_qrParityDecompose(matrix_t* A, matrix_t* Qt, matrix_t* R)
{
  BOOL z_isSuccessfull = TRUE;
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  uint32_t q_k = 0;
  uint32_t q_rowNum = A->row;
  uint32_t q_colNum = A->col;
  double d_sigmaK = 0.0;
  double d_down = 0.0;
  double d_betaK = 0.0;
  double d_gamma = 0.0;
  matrix_t* pz_uK = matrix_new(q_rowNum, 1);
  //Initialization
  for (q_i = 0; q_i < q_rowNum; ++q_i)
  {
    MAT(Qt, q_i, q_i) = 1.0;
    for (q_j = 0; q_j < q_colNum; ++q_j)
    {
      MAT(R, q_i, q_j) = MAT(A, q_i, q_j);
    }
  }
  //end of Initialization
  for (q_k = 0; q_k < q_colNum; ++q_k)
  {
    d_sigmaK = 0.0;
    for (q_i = q_k; q_i < q_rowNum; q_i++)
    {
      if (0.0 == MAT(R, q_i, q_k))
      {
        continue;
      }
      d_sigmaK += (MAT(R, q_i, q_k) * MAT(R, q_i, q_k));
    }
    d_sigmaK = sqrt(d_sigmaK);
    if (MAT(R, q_k, q_k) >= 0.0)
    {
      d_sigmaK = -d_sigmaK;
    }
    for (q_i = 0; q_i < q_rowNum; q_i++)
    {
      if (q_i < q_k)
      {
        MAT(pz_uK, q_i, 0) = 0.0;
      }
      else if (q_i == q_k)
      {
        MAT(pz_uK, q_i, 0) = MAT(R, q_k, q_k) - d_sigmaK;
      }
      else if (q_i > q_k)
      {
        MAT(pz_uK, q_i, 0) = MAT(R, q_i, q_k);
      }
    }
    d_down = d_sigmaK * MAT(pz_uK, q_k, 0);
    if (fabs(d_down) < 1.0e-20)
    {
      z_isSuccessfull = FALSE;
      break;
    }
    d_betaK = -1.0 / (d_down);
    //get the R matrix
    for (q_j = 0; q_j < q_colNum; q_j++)
    {
      d_gamma = 0.0;
      for (q_i = q_k; q_i < q_rowNum; q_i++)
      {
        if (0.0 == MAT(R, q_i, q_j))
        {
          continue;
        }
        d_gamma += (MAT(pz_uK, q_i, 0) * MAT(R, q_i, q_j));
      }
      d_gamma *= d_betaK;
      for (q_i = q_k; q_i < q_rowNum; q_i++)
      {
        MAT(R, q_i, q_j) -= (d_gamma * MAT(pz_uK, q_i, 0));
      }
    }
    for (q_i = q_k + 1; q_i < q_rowNum; q_i++)
    {
      MAT(R, q_i, q_k) = 0.0;
    }
    //end of get the R matrix
    //get the Qt matrix
    for (q_j = 0; q_j < q_rowNum; q_j++)
    {
      d_gamma = 0.0;
      for (q_i = q_k; q_i < q_rowNum; q_i++)
      {
        if (0.0 == MAT(Qt, q_i, q_j))
        {
          continue;
        }
        d_gamma += (MAT(pz_uK, q_i, 0) * MAT(Qt, q_i, q_j));
      }
      d_gamma *= d_betaK;
      for (q_i = q_k; q_i < q_rowNum; q_i++)
      {
        MAT(Qt, q_i, q_j) -= (d_gamma * MAT(pz_uK, q_i, 0));
      }
    }
    //end of get the Qt matrix
  }
  matrix_free(&pz_uK);
  return z_isSuccessfull;
}

/**
 * @brief Matrix Inverse
 * @param[in] mat
 * @param[out] mat_inv
 * @return BOOL
 */
BOOL matrix_inverse(const matrix_t* mat, matrix_t* mat_inv)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t dim = 0;
  double d_Swap = 0.0;
  double d_Pivot = 0.0;
  uint8_t* u_Column_swap = NULL;
  double* d_M = NULL;
  double* d_InvM = NULL;

  if ((mat->col != mat->row) ||
    (mat_inv->col != mat_inv->row) ||
    (mat->col != mat_inv->col) ||
    (0 == mat->col))
  {
    return FALSE;
  }

  dim = mat->col;
  u_Column_swap = (uint8_t*)OS_MALLOC_FAST(dim * sizeof(uint8_t));
  d_M = mat->data;
  d_InvM = mat_inv->data;

  /* Copy d_M into d_InvM so as not to overwrite the input parameter */
  for (i = 0; i < dim; i++)
  {
    for (j = 0; j < dim; j++)
    {
      d_InvM[i * dim + j] = d_M[i * dim + j];
    }
  }

  for (i = 0; i < dim; i++)
  {
    u_Column_swap[i] = i;
  }

  for (i = 0; i < dim; i++)
  {
    d_Pivot = 0.0;
    k = i;

    /* Find the element in this row with the largest absolute value -
      the pivot. Pivoting helps avoid division by small quantities. */
    for (j = i; j < dim; j++)
    {
      if (fabs(d_Pivot) < fabs(d_InvM[i * dim + j]))
      {
        k = j;
        d_Pivot = d_InvM[i * dim + j];
      }
    }

    /* Swap columns if necessary to make the pivot be the leading non-zero
      element of the row. */
    if (i != k)
    {
      j = u_Column_swap[k];
      u_Column_swap[k] = u_Column_swap[i];
      u_Column_swap[i] = j;

      for (j = 0; j < dim; j++)
      {
        d_Swap = d_InvM[j * dim + i];
        d_InvM[j * dim + i] = d_InvM[j * dim + k];
        d_InvM[j * dim + k] = d_Swap;
      }
    }

    /* Divide the row by the pivot, making the leading element 1.0 and
      multiplying the determinant by the pivot. */
    d_InvM[i * dim + i] = 1.0;

    if (fabs(d_Pivot) < 1E-18)
    {
      OS_FREE(u_Column_swap);
      return (FALSE);    /* Pivot = 0 therefore singular matrix. */
    }

    for (k = 0; k < dim; k++)
    {
      d_InvM[i * dim + k] /= d_Pivot;
    }

    /* Gauss-Jordan elimination.  Subtract the appropriate multiple of the
      current row from all subsequent rows to get the matrix into row echelon
      form. */
    for (k = 0; k < dim; k++)
    {
      if (i == k)
      {
        continue;
      }

      d_Pivot = d_InvM[k * dim + i];

      if (d_Pivot == 0.0)
      {
        continue;
      }

      d_InvM[k * dim + i] = 0.0;

      for (j = 0; j < dim; j++)
      {
        d_InvM[k * dim + j] -= (d_Pivot * d_InvM[i * dim + j]);
      }
    }
  }

  /* Swap the columns back into their correct places. */
  for (i = 0; i < dim - 1; i++)
  {
    if (u_Column_swap[i] == i)
    {
      continue;
    }

    /* Find out where the column wound up after column swapping. */
    for (k = i + 1; k < dim; k++)
    {
      if (u_Column_swap[k] == i)
      {
        break;
      }
    }

    if (k < dim)
    {
      u_Column_swap[k] = u_Column_swap[i];
    }
    else
    {
      OS_FREE(u_Column_swap);
      return FALSE;
    }

    for (j = 0; j < dim; j++)
    {
      d_Pivot = d_InvM[i * dim + j];
      d_InvM[i * dim + j] = d_InvM[k * dim + j];
      d_InvM[k * dim + j] = d_Pivot;
    }

    u_Column_swap[i] = i;
  }
  OS_FREE(u_Column_swap);
  return TRUE;
}


/**
 * @brief Right Up Matrix Inverse.
          Right Up Matrix inverse is much simpler than general matrix
 * @param[in] mat
 * @param[out] mat_inv
 * @return None
 */
BOOL matrix_rightup_inverse(matrix_t* mat, matrix_t* mat_inv)
{
  int32_t dim = mat->col;

  for (int32_t i = 0; i < dim; i++)
  {
    if (DBL_EQU(MAT(mat, i, i), DBL_EPSILON))
    {
      return FALSE;
    }
    mat_inv->data[i * dim + i] = 1 / mat->data[i * dim + i];
  }
  for (int32_t i = dim - 2; i >= 0; i--)
  {
    for (int32_t j = i + 1; j <= dim - 1; j++)
    {
      double sum = 0.0;
      for (int32_t k = i + 1; k <= j; k++)
      {
        sum += mat->data[i * dim + k] * mat_inv->data[k * dim + j];
      }
      mat_inv->data[i * dim + j] = -(sum / mat->data[i * dim + i]);
    }
  }

  return TRUE;
}


/**
 * @brief Matrix Print
 * @param[in] mat
 * @return None
 */
void matrix_print(matrix_t* mat)
{
  uint32_t row = mat->row;
  uint32_t col = mat->col;

  printf("Matrix print row=%u col=%u\n", row, col);
  for (uint32_t i = 0; i < row; i++)
  {
    for (uint32_t j = 0; j < col; j++)
    {
      printf("%12.6f ", MAT(mat, i, j));
    }
    printf("\n");
  }
}

/**
 * @brief Create a new upper triangular matrix by dynamic memory allocate
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @return matrix_t - matrix_t pointer
 */
matrix_tri_t* matrix_tri_new(uint32_t dim)
{
  matrix_tri_t* mat_uptri = (matrix_tri_t*)OS_MALLOC(sizeof(matrix_t));
  if (NULL == mat_uptri)
  {
    return NULL;
  }

  uint32_t sz = dim * (dim + 1) /2;
  mat_uptri->dim = dim;
  mat_uptri->data = (void*)OS_MALLOC(sz * sizeof(double));
  if (NULL == mat_uptri->data)
  {
    OS_FREE(mat_uptri);
    mat_uptri = NULL;
    return NULL;
  }
  memset(mat_uptri->data, 0, sz * sizeof(double));

  return mat_uptri;
}

/**
 * @brief Get a element address in a upper triangular matrix
 * @param[in] m - upper triangular matrix
 * @param[in] x
 * @param[in] y
 * @return pointer to field
 */
double* matrix_tri_get(matrix_tri_t* m, uint32_t x, uint32_t y)
{
  if (!(x < (m->dim) && y < (m->dim)))
  {
    return &(m->data[0]);
  }

  uint32_t idx = 0;
  if (x >= y)
  {
    idx = (y + 1) * y / 2 + x;
  }
  else
  {
    idx = (x + 1) * x / 2 + y;
  }

  return &(m->data[idx]);
}

/**
 * @brief Create a new upper triangular matrix from input array
 * @param[in] row - matrix row number
 * @param[in] col - matrix column number
 * @param[in] array - input array
 * @return matrix_t - matrix_t pointer
 */
matrix_tri_t* matrix_tri_new_from_array(uint32_t dim, double* array[])
{
  matrix_tri_t* mat_uptri = matrix_tri_new(dim);
  if (NULL == mat_uptri)
  {
    return NULL;
  }

  for (uint32_t i = 0; i < dim; i++)
  {
    for (uint32_t j = i; j < dim; j++)
    {
      MAT_TRI(mat_uptri, i, j) = array[i][j];
    }
  }

  return mat_uptri;
}

/**
 * @brief Free a upper triangular matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_tri_free(matrix_tri_t* mat)
{
  if (NULL == mat)
  {
    return;
  }

  if (mat->data)
  {
    OS_FREE(mat->data);
    mat->data = NULL;
  }

  OS_FREE(mat);
  mat = NULL;
}

/**
 * @brief Convert a upper triangular matrix to square matrix
 * @param[in] mat_uptri
 * @param[out] mat - square matrix
 * @return None
 */
BOOL matrix_cvt_tri2square(matrix_tri_t* mat_uptri, matrix_t* mat)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(4, mat_uptri, mat_uptri->data,
    mat, mat->data))
  {
    return FALSE;
  }
  
  if ((mat_uptri->dim == mat->col) &&
     (mat->col == mat->row))
  {
    return FALSE;
  }

  for (i = 0; i < mat_uptri->dim; i++)
  {
    for (j = i; j < mat_uptri->dim; j++)
    {
      MAT(mat, i, j) = MAT_TRI(mat_uptri, i, j);
    }
  }
  
  return TRUE;
}

/**
 * @brief Convert a square matrix to a upper triangular matrix
 * @param[in] mat - square matrix
 * @param[out] mat_uptri
 * @return TRUE - Success
           FALSE - Fail
 */
BOOL matrix_cvt_square2tri(matrix_t* mat, matrix_tri_t* mat_uptri)
{
  uint32_t i = 0;
  uint32_t j = 0;
  if (any_Ptrs_Null(4, mat_uptri, mat_uptri->data,
    mat, mat->data))
  {
    return FALSE;
  }

  if ((mat_uptri->dim == mat->col) &&
    (mat->col == mat->row))
  {
    return FALSE;
  }
  
  for (i = 0; i < mat_uptri->dim; i++)
  {
    for (j = i; j < mat_uptri->dim; j++)
    {
      MAT_TRI(mat_uptri, i, j) = MAT(mat, i, j);
    }
  }

  return TRUE;
}

/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : matrix_t *H        I   (weighted) design matrix (n x m)
*          matrix_t *y        I   (weighted) measurements (n x 1)
*          matrix_t *x        O   estmated parameters (m x 1)
*          matrix_t *Q        O   esimated parameters covariance matrix (m x m)
* return : status (1:ok,0:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*-----------------------------------------------------------------------------*/
BOOL lsqMat(const matrix_t* H, const matrix_t* y, matrix_t* x, matrix_t* Q)
{
  matrix_t* Hy;
  matrix_t* HT;
  matrix_t* HTH;
  uint32_t n = H->row;
  uint32_t m = H->col;

  if (n < m) return FALSE;
  Hy = matrix_new(m, 1);
  HT = matrix_clone(H);
  HTH = matrix_clone(Q);
  matrix_transpose(HT);
  matrix_mul(HT, y, Hy);
  matrix_mul(HT, H, HTH);
  if (matrix_inverse(HTH, Q))
  {
    matrix_mul(Q, Hy, x);
  }
  else
  {
    matrix_free(&Hy);
    matrix_free(&HT);
    matrix_free(&HTH);
    return FALSE;
  }
  matrix_free(&Hy);
  matrix_free(&HT);
  matrix_free(&HTH);

  return TRUE;
}


/**
 * @brief Sort general function based on Insert Sort Algorithm
 * @param[in/out] data
 * @param[in]   step - each data step
 * @param[in]   count - data count
 * @param[in]   compare - compare funtion
 * @return      None
 */
void loc_sort(uint8_t* data, uint8_t step, uint8_t count, BOOL(*compare)(void*, void*))
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint32_t select_id = 0;
  uint8_t* temp = (uint8_t*)OS_MALLOC(step*sizeof(uint8_t));
  
  if (NULL == temp)
  {
    return;
  }

  for (i = 0; i < (uint8_t)(count - 1); i++)
  {
    select_id = i;
    for (j = i + 1; j < count; j++)
    {
      if (compare(data + j * step, data + select_id * step))
      {
        select_id = j;
      }
    }
    if (i == select_id)
    {
      continue;
    }
    memcpy(temp,                    data + i * step,         step);
    memcpy(data + i * step,         data + select_id * step, step);
    memcpy(data + select_id * step, temp,                    step);
  }
  
  OS_FREE(temp);
  temp = NULL;
}

/**
 * @brief computes int8_t type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
int32_t vector_u8_sum(const uint8_t* u_vals, int32_t q_nums)
{
  if (NULL == u_vals)
  {
    return 0;
  }

  int32_t sum = 0;
  for (int32_t i = 0; i < q_nums; i++)
  {
    sum += u_vals[i];
  }
  return sum;
}

/**
 * @brief computes int32_t type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
int32_t vector_q32_sum(const int32_t* q_vals, int32_t q_nums)
{
  if (NULL == q_vals)
  {
    return 0;
  }

  int32_t sum = 0;
  for (int32_t i = 0; i < q_nums; i++)
  {
    sum += q_vals[i];
  }
  return sum;
}

/**
 * @brief Max value in double type array
 * @param[in] pd_data
 * @param[in] q_num
 * @return max value in double type array
 */
double vector_dbl_max(const double* pd_data, uint32_t q_num)
{
  if (NULL == pd_data)
  {
    return 0.0;
  }
  double d_max = pd_data[0];
  for (uint32_t i = 1; i < q_num; i++)
  {
    if (pd_data[i] > d_max)
    {
      d_max = pd_data[i];
    }
  }
  return d_max;
}

/**
 * @brief computes float type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
double vector_dbl_min(const double* pd_data, uint32_t q_num)
{
  if (NULL == pd_data)
  {
    return 0.0;
  }
  double d_min = pd_data[0];
  for (uint32_t i = 1; i < q_num; i++)
  {
    if (pd_data[i] < d_min)
    {
      d_min = pd_data[i];
    }
  }
  return d_min;
}

/**
 * @brief computes float type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
float vector_flt_sum(const float* d_vals, int32_t q_nums)
{
  if (NULL == d_vals)
  {
    return 0.0;
  }

  float sum = 0;
  for (int32_t i = 0; i < q_nums; i++)
  {
    sum += d_vals[i];
  }
  return sum;
}

/**
 * @brief computes double type summary of a vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      summary of a vector
 */
double vector_dbl_sum(const double* d_vals, int32_t q_nums)
{
  if (NULL == d_vals)
  {
    return 0.0;
  }

  double sum = 0;
  for (int32_t i = 0; i < q_nums; i++)
  {
    sum += d_vals[i];
  }
  return sum;
}

/**
 * @brief computes the 2-norm of an arbitrary length vector
 * @param[in]   d_vals - value array
 * @param[in]   q_nums - array count
 * @return      the 2-norm of the vector
 */
double vector_norm(const double* d_vals, uint32_t q_nums)
{
  uint32_t q_i;
  double d_Sum = 0.0;
  for (q_i = 0; q_i < q_nums; q_i++)
  {
    d_Sum += (*(d_vals + q_i)) * (*(d_vals + q_i));
  }
  return sqrt(d_Sum);
}

/**
 * @brief Free all matrix memory
 * @param[in] mat
 * @return None
 */
void matrix_free_batch(int32_t l_NumPtr, ...)
{
  va_list z_Args;
  va_start(z_Args, l_NumPtr);

  for (int32_t l_PtrIdx = 0; l_PtrIdx < l_NumPtr; l_PtrIdx++)
  {
    matrix_free(va_arg(z_Args, matrix_t**));
  }
  va_end(z_Args);
}

/**
 * @brief Safe free pointer created by malloc
 * @param[in,out] ptr
 */
void safe_free_p(void *ptr)
{
  if (ptr != NULL)
  {
    free(ptr);
    ptr = NULL;
  }
  return;
}

/**
 * @brief Get sum of '1' in binary format marklist for specific LSB
 * @param[in] t_src marklist
 * @param[in] u_lsbwide LSB length
 * @return uint8_t count
 */
uint8_t sum_of_bit(uint64_t t_src, uint8_t u_lsbwide)
{
	uint8_t u_cnt = 0;

	for(uint8_t u_i = 0; u_i < u_lsbwide; u_i++)
	{
		if ((t_src >> u_i) & 1)  u_cnt++;
	}

	return u_cnt;
}