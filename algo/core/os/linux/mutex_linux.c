/**@file        mutex_linux.c
 * @brief       mutex source file
 * @details     contain mutex related API on Linux platform
 * @author      caizhijie
 * @date        2022/01/29
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
  pthread_mutex_init(mutex, NULL);
  return TRUE;
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
  pthread_mutex_destroy(mutex);
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
  if (0 != pthread_mutex_lock(mutex))
  {
    return FALSE;
  }
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
  if (0 != pthread_mutex_unlock(mutex))
  {
    return FALSE;
  }
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
  pthread_cond_init(signal,NULL);
  return TRUE;
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
  
  pthread_cond_wait(signal, mutex);
  
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

  pthread_cond_signal(signal);

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
  pthread_cond_destroy(signal);

  return TRUE;
}
