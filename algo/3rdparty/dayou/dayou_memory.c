/**@file        dayou_memory.c
 * @brief       
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * </table>
 *
 **********************************************************************************
 */

#include <string.h>
#include "dayou_memory.h"


int dayou_memory_init()
{
  return 1;
}

void* dayou_memory_alloc(unsigned int size)
{
  return malloc(size);
}

int dayou_memory_free(void** ptr)
{
  if (ptr == NULL)
  {
    return 0;
  }

  if (*ptr == NULL)
  {
    return 0;
  }
  free(*ptr);
  *ptr = NULL;

  return 1;
}