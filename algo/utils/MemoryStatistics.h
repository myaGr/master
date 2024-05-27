/**@file        MemoryStatistics.h
 * @brief       Memory statistics tool header file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/10/30  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __MEMORY_STAT_H__
#define __MEMORY_STAT_H__

#ifdef __cplusplus
extern "C" {
#endif

void  MemoryStatisticsEnable();
void  MemoryStatisticsDisable();
void* MemoryStatisticsMalloc(int sz, const char* func, int line, const char* file);
void  MemoryStatisticsFree(void* ptr);
void  MemoryStatisticsPrint(char* filepath);

#ifdef __cplusplus
}
#endif

#endif
