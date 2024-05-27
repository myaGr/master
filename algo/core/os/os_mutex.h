/**@file        os_mutex.h
 * @brief       mutex header file
 * @details     contain mutex related API
 * @author      caizhijie
 * @date        2022/01/29
 * @version     V0.1
 **********************************************************************************
 * @note        Implement depends on system platform specific macro
                Windows: _MSC_VER
                Linux  : __gnu_linux__
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/29  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __OS_MUTEX_H__
#define __OS_MUTEX_H__

#include "cmn_def.h"

#if defined(_MSC_VER)
/** Windows Visual Studio platform */
#include <windows.h>
typedef CRITICAL_SECTION    mutex_t;
typedef CONDITION_VARIABLE  signal_t;

#elif defined(__gnu_linux__)

#include <pthread.h>
typedef pthread_mutex_t     mutex_t;
typedef pthread_cond_t      signal_t;

#else

typedef struct{
	void* handle;
	uint32_t   value;
 } mutex_t;
   
typedef struct{
	void* handle;
	uint32_t   value;
 } signal_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * create a mutex
 * @param[out] mutex: handle returned by mutex_create
 * @param[in]  name: mutex's name
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_init(mutex_t* mutex, const char* name);

/**
 * @brief: delete a mutex
 * @param[in] mutex: the mutex need to be deleted.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_destroy(mutex_t* mutex);

/**
 * @brief: take a mutex
 * @param[in] mutex: the mutex need to be take.
 * @param[in] millisec: timeout millisec.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_lock(mutex_t* mutex);

/**
 * @brief: release a mutex
 * @param[in] mutex: the mutex need to be release.
 * @return:
 *  TRUE - success
 *  FALSE - fail for any reason
 */
BOOL mutex_unlock(mutex_t* mutex);

/**
 *  @brief  signal variable initialize
 *  @param[in]   signal: pointer to uninitialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_init(signal_t* signal);

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
BOOL signal_wait(signal_t* signal, mutex_t* mutex, uint32_t tm);

/**
 *  @brief   send out a specific signal value
 *  @param[in]  signal: pointer to initialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_trigger(signal_t* signal);

/**
 *  @brief  delete a specific signal value
 *  @param[in] cond: signal to initialized signal value
 *  @return:
 *    TRUE - success
 *    FALSE - fail for any reason
 */
BOOL signal_destroy(signal_t* signal);

#ifdef __cplusplus
}
#endif

#endif
