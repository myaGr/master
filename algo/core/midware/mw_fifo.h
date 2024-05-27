/**@file        mw_fifo.h
 * @brief       Midware first input first output buffer header file
 * @version     V0.1
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/28  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __MW_FIFO_H__
#define __MW_FIFO_H__

#include "cmn_def.h"

BEGIN_DECL

/** fifo struct */
typedef struct {
  /** fifo input data count */
  size_t  in;

  /** fifo output data count */
  size_t  out;

  /** fifo size  */
  size_t  size;

  /** fifo size  */
  size_t  space;

  /** fifo store memory  */
  uint8_t *buf;
} fifo_t;

/**
 * @brief Create a fifo struct with specified buffer size
 * @param[in]   size_t blksize
 * @param[in]   size_t blknum
 * @param[out] 
 * @return      The pointer to created fifo struct or NULL pointer
 * @note The function will call malloc to alloc memory to fifo struct. 
         So, if the created fifo is useless, it needs to call fifo_destroy 
         to release memory.
*/
fifo_t* fifo_create(size_t blksize, size_t blknum);

/**
 * @brief Destroy a fifo struct
 * @param[in]  fifo struct pointer
 * @param[out] 
 * @return    TRUE - success
 *            FALSE - fail
*/
BOOL fifo_destroy(fifo_t *fifo);

/**
 * @brief Get a fifo used space
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    uesd space size
*/
size_t fifo_used(fifo_t* fifo);

/**
 * @brief Get a fifo remaining space
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    remaining space size
*/
size_t fifo_space(fifo_t *fifo);

/**
 * @brief Clear a fifo, reset the fifo count and memory
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    TRUE - success
 *            FALSE - fail
*/
BOOL fifo_clear(fifo_t *fifo);

/**
 * @brief Push data into fifo
 * @param[in]  fifo - struct pointer
 * @param[in]  buf - data memory should be pushed
 * @param[in]  size - data size
 * @param[out]
 * @return    Pushed data
*/
size_t fifo_push(fifo_t *fifo, const void *buf, size_t size);

/**
 * @brief Pull data from fifo into buf
 * @param[in]  fifo - struct pointer
 * @param[in]  buf - data memory should be pull
 * @param[in]  size - data size
 * @param[out]
 * @return    Pulled data
*/
size_t fifo_pull(fifo_t *fifo, void *buf, size_t size);

END_DECL

#endif /* !__FIFO_H__ */
