/**@file        mw_fifo.c
 * @brief       first input first output buffer module
 * @version     V0.1
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
#include "mw_fifo.h"
#include "mw_alloc.h"

#define FIFO_MALLOC(x) OS_MALLOC(x)
#define FIFO_FREE(x)   OS_FREE(x)

#define FIFO_MIN(x, y)   ((x)<(y)?(x):(y))

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
fifo_t* fifo_create(size_t blksize, size_t blknum)
{
  fifo_t* fifo = NULL;
  uint8_t* buf = NULL;
  size_t sz = blksize * blknum;
  
  fifo = (fifo_t*)FIFO_MALLOC(sizeof(fifo_t));
  if (NULL == fifo)
  {
    return NULL;
  }
  
  buf = (uint8_t*)FIFO_MALLOC(sz);
  if (NULL == buf)
  { 
    FIFO_FREE(fifo);
    return NULL;
  }
  
  fifo->buf = buf;
  fifo->size = sz;
  fifo->in = 0;
  fifo->out = 0;
  
  return fifo;
}

/**
 * @brief Destroy a fifo struct
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    TRUE - success
 *            FALSE - fail
*/
BOOL fifo_destroy(fifo_t *fifo)
{
  if (NULL == fifo)
  {
    return FALSE;
  }
  
  if (NULL != fifo->buf)
  {
     FIFO_FREE(fifo->buf);
  }
  
  FIFO_FREE(fifo);
  
  return TRUE;
}

/**
 * @brief Get a fifo used space
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    uesd space size
*/
size_t fifo_used(fifo_t* fifo)
{
  if (NULL == fifo)
  {
    return 0;
  }

  if (fifo->in >= fifo->out)
  {
    return (fifo->in - fifo->out);
  }
  else
  {
    return (fifo->in + fifo->size - fifo->out);
  }
}

/**
 * @brief Get a fifo remaining space
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    remaining space size
*/
size_t fifo_space(fifo_t *fifo)
{
  if (NULL == fifo)
  {
    return 0;
  }

  return fifo->size - fifo_used(fifo);
}

/**
 * @brief Clear a fifo, reset the fifo count and memory
 * @param[in]  fifo struct pointer
 * @param[out]
 * @return    TRUE - success
 *            FALSE - fail
*/
BOOL fifo_clear(fifo_t *fifo)
{
  if (NULL == fifo)
  {
    return FALSE;
  }
  
  fifo->in = 0;
  fifo->out = 0;
  memset(fifo->buf, 0, fifo->size);
  return TRUE;
}

/**
 * @brief Push data into fifo
 * @param[in]  fifo - struct pointer
 * @param[in]  buf - data memory should be pushed
 * @param[in]  size - data size
 * @param[out]
 * @return    Pushed data
*/
size_t fifo_push(fifo_t *fifo, const void *buf, size_t size)
{
  size_t l = 0;
  if ((NULL == fifo) || (NULL == buf))
  {
    return 0;
  }
  
  fifo->space = fifo_space(fifo);
  if (size >= fifo->space)
  {
    return 0;
  }
  
  if (size > fifo->size - fifo->in)
  {
    l = fifo->size - fifo->in;
    memcpy(fifo->buf + fifo->in, buf, l);
    memcpy((char*)fifo->buf, (char*)buf + l, size - l);
  }
  else
  {
    memcpy(fifo->buf + fifo->in, buf, size);
  }

  fifo->in += size;
  if (fifo->in >= fifo->size)
  {
    fifo->in -= fifo->size;
  }
  fifo->space = fifo_space(fifo);
  return size;
}

/**
 * @brief Pull data from fifo into buf
 * @param[in]  fifo - struct pointer
 * @param[in]  buf - data memory should be pull
 * @param[in]  size - data size
 * @param[out]
 * @return    Pulled data
*/
size_t fifo_pull(fifo_t *fifo, void *buf, size_t size)
{
  size_t l = 0;
  if ((NULL == fifo) || (NULL == buf))
  {
    return 0;
  }
  
  if (size > fifo_used(fifo))
  {
    return 0;
  }

  if (size > fifo->size - fifo->out)
  {
    l = fifo->size - fifo->out;
    memcpy(buf, fifo->buf + fifo->out, l);
    memcpy((char*)buf + l, fifo->buf, size - l);
  }
  else
  {
    memcpy(buf, fifo->buf + fifo->out, size);
  }

  fifo->out += size;
  if (fifo->out >= fifo->size)
  {
    fifo->out -= fifo->size;
  }

  fifo->space = fifo_space(fifo);
  return size;
}
