/**@file        MemoryStatistics.c
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

#include <vector>
#include <map>
#include <string>
#include <list>
#include <algorithm>
#include <chrono>

#include <string.h>

#include "MemoryStatistics.h"

#if defined( _WIN32)
#include <windows.h>
#endif

using namespace std;
using namespace chrono;

class MemoryStatisticsBlock
{
public:
  long long index;
  int    size;
  string func;
  string file;
  int    line;
  char* buffer;
};

class MemoryStatisticsObserver
{
public:
  int current_size;
  int max_size;
  list<MemoryStatisticsBlock> max_case_blocks;
  long long max_index;
  string max_case_file;
  string max_case_func;
  int max_case_line;
};

map<char*, MemoryStatisticsBlock*> memoryBlockPool;
MemoryStatisticsObserver memoryObserver;
bool stat_on = true;

bool sort_func(const MemoryStatisticsBlock& m1, const MemoryStatisticsBlock& m2)
{
  return m1.index > m2.index;
}

void MemoryStatisticsEnable()
{
  stat_on = true;
  return;
}

void MemoryStatisticsDisable()
{
  stat_on = false;
  return;
}

void* MemoryStatisticsMalloc(int sz, const char* func, int line, const char* file)
{
  static long long idx = 0;

  if (!stat_on)
  {
    return malloc(sz);
  }

  MemoryStatisticsBlock* m = new MemoryStatisticsBlock();
  m->index = idx++;
  m->file = string(file);
  m->func = string(func);
  m->line = line;
  m->size = sz;
  m->buffer = new char[sz];
  memoryBlockPool[m->buffer] = m;
  memset(m->buffer, 0, sz);

  memoryObserver.current_size += sz;
  if (memoryObserver.max_size < memoryObserver.current_size)
  {
    memoryObserver.max_size = memoryObserver.current_size;
    memoryObserver.max_case_blocks.clear();
    memoryObserver.max_index = m->index;
    memoryObserver.max_case_file = string(file);
    memoryObserver.max_case_func = string(func);
    memoryObserver.max_case_line = line;

    map<char*, MemoryStatisticsBlock*>::iterator iter;
    for (iter = memoryBlockPool.begin(); iter != memoryBlockPool.end(); iter++)
    {
      memoryObserver.max_case_blocks.push_back(*(iter->second));
    }

    memoryObserver.max_case_blocks.sort(sort_func);
  }

  return m->buffer;
}

void MemoryStatisticsFree(void* ptr)
{
  map<char*, MemoryStatisticsBlock*>::iterator iter = memoryBlockPool.find((char*)ptr);
  if (iter == memoryBlockPool.end())
  {
    if (!ptr)
    {
      free(ptr);
      ptr = NULL;
    }
    return;
  }

  memoryObserver.current_size -= iter->second->size;
  delete[] iter->second->buffer;
  iter->second->buffer = NULL;
  delete iter->second;
  iter->second = NULL;

  memoryBlockPool.erase(iter);

  return;
}

void MemoryStatisticsPrint(char* filepath)
{
  static int max_size_store = 0;

  if (memoryObserver.max_size < max_size_store)
  {
    return;
  }

  list<MemoryStatisticsBlock> all_info;

  map<char*, MemoryStatisticsBlock*>::iterator iter;
  for (iter = memoryBlockPool.begin(); iter != memoryBlockPool.end(); iter++)
  {
    all_info.push_back(*(iter->second));
  }

  all_info.sort(sort_func);

  FILE* fp = NULL;
  if (filepath == NULL)
  {
    fp = fopen("memory.txt", "w+");
  }
  else
  {
    fp = fopen(filepath, "w+");
  }

  fprintf(fp, "current memory size %d\n", memoryObserver.current_size);
  for (list<MemoryStatisticsBlock>::iterator iter2 = all_info.begin(); iter2 != all_info.end(); iter2++)
  {
    fprintf(fp, "Size: %8d Func: %20s, line:%4d, %20s\n",
      iter2->size, iter2->func.c_str(), iter2->line, iter2->file.c_str());
  }

  fprintf(fp, "max memory size %d \n", memoryObserver.max_size);
  fprintf(fp, "max happen at index %8lld %20s, line:%6d, %20s\n", memoryObserver.max_index,
    memoryObserver.max_case_func.c_str(), memoryObserver.max_case_line,
    memoryObserver.max_case_file.c_str());
  for (list<MemoryStatisticsBlock>::iterator iter2 = memoryObserver.max_case_blocks.begin(); iter2 != memoryObserver.max_case_blocks.end(); iter2++)
  {
    fprintf(fp, "index %8lld size: %8d func: %20s, line:%6d, %20s\n", iter2->index,
      iter2->size, iter2->func.c_str(), iter2->line, iter2->file.c_str());
  }

  max_size_store = memoryObserver.max_size;
  fclose(fp);
  return;
}
