/**@file        thread_linux.c
 * @brief       thread Implement on Linux OS
 * @details     contain thread related API
 * @author      caizhijie
 * @date        2022/01/29
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        Implement depends on system platform specific macro 
                Linux: __gnu_linux__
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Author      <th>Description
 * <tr><td>2022/01/29  <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "os_thread.h"

#include <unistd.h>

 /**
  * @brief  Create a thread with args
  * @param[io] thread_id
  * @param[in] task_name
  * @param[in] entryProc
  * @param[in] args
  * @param[in] stackPtr
  * @param[in] stackSize
  * @param[in] prio
  * @return TRUE -  success
  *         FALSE - fail for any reason
  * @note   The function provide cross platform implement.
  */
BOOL thread_create( thread_id_t* p_thread_id,
                    const char* task_name,
                    thread_proc_t entry_proc,
                    void* args,
                    thread_stk_t* p_stack,
                    size_t stack_size,
                    uint8_t u_prio)
{
  pthread_create(p_thread_id, NULL, entry_proc, NULL);
  return TRUE;
}

/**
 * @brief  Wait for a thread quite
 * @param[in] p_thread_id
 * @param[in] q_timeout
 * @note   The function provide cross platform implement.
 */
void thread_join(thread_id_t* p_thread_id, uint32_t q_timeout)
{
  if (NULL == p_thread_id)
  {
    return;
  }
  
  pthread_join(*p_thread_id, NULL);
  
  return;
}

/**
 * @brief     Thread sleep function
 * @param[in] q_tm
 * @note      The function sleep for q_tm millisecond
 */
void thread_sleep(uint32_t q_tm)
{
  usleep(q_tm * 1000);
}

/**
 * @brief     Get thread id
 * @return    thread unique id
 */
uint32_t get_thread_id()
{
  return (uint32_t)pthread_self();
}