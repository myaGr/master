/**@file        mutex_win.c
 * @brief       mutex source file
 * @details     contain mutex related API on Windows platform
 * @author      caizhijie
 * @date        2022/01/29
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022 Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note        Implement depends on system platform specific macro 
                Windows: _MSC_VER
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

/*
 * create a mutex
 * @param[out] mutex: handle returned by mutex_create
 * @param[in]  name: mutex's name
 * @return:
 *  true - success
 *  false - fail for any reason
 */
BOOL mutex_init(mutex_t* mutex, const char* name)
{
  InitializeCriticalSection(mutex);
  return TRUE;
}

/*
 * @brief: delete a mutex
 * @param[in] mutex: the mutex need to be deleted.
 * @return:
 *  true - success
 *  false - fail for any reason
 */
BOOL mutex_destroy(mutex_t* mutex)
{
  if(NULL == mutex) {
      return FALSE;
  }

  DeleteCriticalSection(mutex);
  return TRUE;
}

/*
 * @brief: take a mutex
 * @param[in] mutex: the mutex need to be take.
 * @param[in] millisec: timeout millisec.
 * @return:
 *  true - success
 *  false - fail for any reason
 */
BOOL mutex_lock(mutex_t* mutex)
{
  if(!mutex) {
      return FALSE;
  }

  EnterCriticalSection(mutex);
  return TRUE;
}

/*
 * @brief: release a mutex
 * @param[in] mutex: the mutex need to be release.
 * @return:
 *  true - success
 *  false - fail for any reason
 */
BOOL mutex_unlock(mutex_t* mutex)
{
  if (!mutex) {
    return FALSE;
  }

  LeaveCriticalSection(mutex);

  return TRUE;
}

/**
 *  @brief  signal variable initialize
 *  @param[in]   signal: pointer to uninitialized signal value
 *  @return:
 *    true - success
 *    false - fail for any reason
 */
BOOL signal_init(signal_t* signal)
{
  if (NULL == signal) {
    return FALSE;
  }

  InitializeConditionVariable(signal);

  return TRUE;
}

/**
 *  @brief  wait for a signal trigger
 *  @param[in] signal: pointer to initialized condition value
 *  @param[in] mutex: pointer to initialized mutex value
 *  @param[in] tm: timeout limit.  0 : wait infinite
 *                                >0 : wait tm millisecond
 *  @return:
 *    true - success
 *    false - fail for any reason
 */
BOOL signal_wait(signal_t* signal, mutex_t* mutex, uint32_t tm)
{
  if ((NULL == signal) || (NULL == mutex))
  {
    return FALSE;
  }

  if (tm == 0) {
    SleepConditionVariableCS(signal, mutex, INFINITE);
  }
  else {
    SleepConditionVariableCS(signal, mutex, tm);
  }
  return TRUE;
}

/**
 *  @brief   send out a specific signal value
 *  @param[in]  signal: pointer to initialized signal value
 *  @return:
 *    true - success
 *    false - fail for any reason
 */
BOOL signal_trigger(signal_t* signal)
{
  if (NULL == signal) {
    return FALSE;
  }

  WakeConditionVariable(signal);

  return TRUE;
}

/**
 *  @brief  destroy a specific signal value
 *  @param[in] cond: signal to initialized signal value
 *  @return:
 *    true - success
 *    false - fail for any reason
 */
BOOL signal_destroy(signal_t* signal)
{
  return TRUE;
}
