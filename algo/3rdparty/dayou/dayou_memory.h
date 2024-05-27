/**@file        dayou_memory.h
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

#ifndef __DAYOU_MEMORY_H__
#define __DAYOU_MEMORY_H__

int dayou_memory_init();

void* dayou_memory_alloc(unsigned int size);

int dayou_memory_free(void** ptr);


#endif
