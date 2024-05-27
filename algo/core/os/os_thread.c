/**@file        os_thread.cpp
 * @brief       thread port source file
 * @details     contain thread related API
 * @author      caizhijie
 * @date        2022/01/21
 * @version     V0.1
 **********************************************************************************
 * @note        Implement depends on system platform specific macro 
                Windows: _MSC_VER
                Linux  : __gnu_linux__
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "os_thread.h"
#if defined(_MSC_VER)

#include "windows/thread_win.c"

#elif defined(__gnu_linux__)

#include "linux/thread_linux.c"

#else

BOOL thread_create(thread_id_t* p_thread_id,
  const char* task_name,
  thread_proc_t entry_proc,
  void* args,
  thread_stk_t* p_stack,
  size_t stack_size,
  uint8_t u_prio)
{
  return TRUE;
}

void thread_join(thread_id_t* p_thread_id, uint32_t q_timeout)
{
  return;
}

void thread_sleep(uint32_t q_tm)
{
  return;
}

uint32_t get_thread_id()
{
  return 0;
}

#endif