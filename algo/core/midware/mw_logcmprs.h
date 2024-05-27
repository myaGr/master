/**@file        mw_logcmprs.h
 * @brief       log compress header file
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

#ifndef __MW_COMPRESS_H__
#define __MW_COMPRESS_H__

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdint.h>

#include "cmn_def.h"

#include "quicklz.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief De-Initialize compress module
 * @return: TRUE  - Init Success
 *          FALSE - Init Fail
 */
BOOL mw_CompressRelease();

/*
 * @brief compress log
 * @param[in] src           -the address of log need compress
 * @param[in] src_len       -the length of the log
 * @param[in] output_buffer -the buffer store log compressed (suggest space's q_size equal to the paramter of function mw_CompressInit)
 * @return: output_len      -store length of log compressed
 */
uint32_t mw_Compress(const uint8_t* src, uint32_t src_len, uint8_t* output_buffer);

/*
 * @brief decompress log
 * @param[in] src           -the address of log need compress
 * @param[in] output_buffer -the buffer store log compressed (suggest space's q_size equal to the paramter of function mw_CompressInit)
 * @return:   output_len    -store length of log decompressed
 */
uint32_t mw_Decompress(const uint8_t* src, uint8_t* output_buffer);

#ifdef __cplusplus
}
#endif

#endif