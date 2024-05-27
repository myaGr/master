/**@file        mw_logcmprs.c
 * @brief       log compress implement file
 * @Detail      
 * @author      caizhijie
 * @date        2023/08/07
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/07  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "mw_logcmprs.h"
#include "mw_alloc.h"

typedef struct compress_worker_t
{
  uint8_t               u_init;
  qlz_state_compress*   state_compress;
  qlz_state_decompress* state_decompress;
} compress_worker_t;

static compress_worker_t g_compress_worker = { 0 };

/**
 * @brief Initialize compress module 
 * @param[in] q_size : 
 * @return: TRUE  - Init Success
 *          FALSE - Init Fail
 */
static BOOL mw_CompressInit()
{
  if (TRUE == g_compress_worker.u_init)
  {
    return TRUE;
  }

  g_compress_worker.state_compress = (qlz_state_compress*)OS_MALLOC(sizeof(qlz_state_compress));
  if (NULL == g_compress_worker.state_compress)
  {
    return FALSE;
  }

  g_compress_worker.state_decompress = (qlz_state_decompress*)OS_MALLOC(sizeof(qlz_state_decompress));
  if (NULL == g_compress_worker.state_decompress)
  {
    return FALSE;
  }

  g_compress_worker.u_init = TRUE;

  return TRUE;
}

/**
 * @brief De-Initialize compress module
 * @param[in] q_size :
 * @return: TRUE  - Init Success
 *          FALSE - Init Fail
 */
BOOL mw_CompressRelease()
{
  if (FALSE == g_compress_worker.u_init)
  {
    return FALSE;
  }

  OS_FREE(g_compress_worker.state_compress);
  OS_FREE(g_compress_worker.state_decompress);

  return TRUE;
}

/*
 * @brief compress log
 * @param[in] src           -the address of log need compress
 * @param[in] src_len       -the length of the log
 * @param[in] output_buffer -the buffer store log compressed (suggest space's q_size equal to the paramter of function mw_CompressInit)
 * @return: output_len      -store length of log compressed
 */
uint32_t mw_Compress(const uint8_t* src, uint32_t src_len, uint8_t* output_buffer)
{
  if (FALSE == g_compress_worker.u_init)
  {
    if (FALSE == mw_CompressInit())
    {
      return FALSE;
    }
  }

  memset(g_compress_worker.state_compress, 0, sizeof(qlz_state_compress));
  uint32_t output_len = qlz_compress((const void*)src, (char*)output_buffer,
    src_len, g_compress_worker.state_compress);

  return output_len;
}

/*
 * @brief decompress log
 * @param[in] src           -the address of log need compress
 * @param[in] output_buffer -the buffer store log compressed (suggest space's q_size equal to the paramter of function mw_CompressInit)
 * @return:   output_len    -store length of log decompressed
 */
uint32_t mw_Decompress(const uint8_t* src, uint8_t* output_buffer)
{
  if (FALSE == g_compress_worker.u_init)
  {
    if (FALSE == mw_CompressInit())
    {
      return FALSE;
    }
  }

  memset(g_compress_worker.state_decompress, 0, sizeof(qlz_state_decompress));
  uint32_t output_len = qlz_decompress((const char*)src, (char*)output_buffer,
    g_compress_worker.state_decompress);

  return output_len;
}
