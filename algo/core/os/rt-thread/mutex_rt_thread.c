/**@file        mutex_freertos.c
 * @brief       mutex source file
 * @details     contain mutex related API on Linux platform
 * @author      zhanglei
 * @date        2022/06/20
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022 Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        Implement depends on system platform specific macro 
                Linux: __gnu_linux__
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/29  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "os_mutex.h"
#include <rtthread.h>
/**
 * create a mutex
 * @param[out] mutex: handle returned by mutex_create
 * @param[in]  name: mutex's name
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_init(mutex_t* mutex, const char* name)
{
  if (NULL == mutex) 
  {
    return FALSE;
  }
  mutex->handle = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
  if(NULL != mutex->handle) {
      return TRUE;
  } else {
	  return FALSE;
  }
}

/**
 * @brief: delete a mutex
 * @param[in] mutex: the mutex need to be deleted.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_destroy(mutex_t* mutex)
{
  if (NULL == mutex) 
  {
    return FALSE;
  }
  rt_mutex_delete(mutex->handle);
  mutex->handle = NULL;
  return TRUE;
}

/**
 * @brief: take a mutex
 * @param[in] mutex: the mutex need to be take.
 * @param[in] millisec: timeout millisec.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_lock(mutex_t* mutex)
{
  if (NULL == mutex) {
    return FALSE;
  }  
  rt_mutex_take(mutex->handle, RT_WAITING_FOREVER);
  return TRUE;
}

/**
 * @brief: release a mutex
 * @param[in] mutex: the mutex need to be release.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_unlock(mutex_t* mutex)
{
  if (NULL == mutex) {
    return FALSE;
  }  
  rt_mutex_release(mutex->handle);
  return TRUE;
}

/**
 *  @brief  signal variable initialize
 *  @param[in]   signal: pointer to uninitialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_init(signal_t* signal)
{
  if (NULL == signal) {
    return FALSE;
  }
  signal->handle = rt_sem_create(NULL, 0, RT_IPC_FLAG_PRIO);
  if(NULL != signal->handle) {
      return TRUE;
  } else {
	  return FALSE;
  }
}

/**
 *  @brief  wait for a signal trigger
 *  @param[in] signal: pointer to initialized condition value
 *  @param[in] mutex: pointer to initialized mutex value
 *  @param[in] tm: timeout limit.  0 : wait infinite
 *                                >0 : wait tm millisecond
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_wait(signal_t* signal, mutex_t* mutex, uint32_t tm)
{
  if ((NULL == signal) || (NULL == mutex))
  {
    return FALSE;
  }
  rt_mutex_release(mutex->handle);
  rt_sem_take(signal->handle,RT_WAITING_FOREVER);
  rt_mutex_take(mutex->handle, RT_WAITING_FOREVER);
  return TRUE;
}

/**
 *  @brief   send out a specific signal value
 *  @param[in]  signal: pointer to initialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_trigger(signal_t* signal)
{
  if (NULL == signal) {
    return FALSE;
  }
  rt_sem_release(signal->handle);
  return TRUE;
}

/**
 *  @brief  destroy a specific signal value
 *  @param[in] cond: signal to initialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_destroy(signal_t* signal)
{
  rt_sem_delete(signal->handle);
  signal->handle = NULL;
  return TRUE;
}
