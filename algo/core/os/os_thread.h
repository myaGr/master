/**@file        os_thread.h
 * @brief       thread port header file
 * @details     contain thread related API
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
 * <tr><td>2022/01/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */


#ifndef __OS_THREAD_H__
#define __OS_THREAD_H__

#include "cmn_def.h"

#if defined(_MSC_VER)

#include <windows.h>

typedef HANDLE      thread_id_t;
typedef uint32_t    thread_stk_t;

#elif defined(__gnu_linux__)

#include <pthread.h>
typedef pthread_t   thread_id_t;
typedef uint32_t    thread_stk_t;

#else

typedef struct{
	void* thread;
	uint32_t   value;
} thread_id_t;

typedef uint32_t    thread_stk_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/** thread entry proc type definition */
typedef void* (*thread_proc_t)(void*);

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
                    uint8_t u_prio);

/**
 * @brief  Wait for a thread quite
 * @param[in] p_thread_id
 * @param[in] q_timeout
 * @note   The function provide cross platform implement.
 */
void thread_join( thread_id_t* p_thread_id, uint32_t q_timeout);

/**
 * @brief     Thread sleep function
 * @param[in] q_tm
 * @note      The function sleep for q_tm millisecond
 */
void thread_sleep(uint32_t q_tm);

/**
 * @brief     Get thread id
 * @return    thread unique id
 */
uint32_t get_thread_id(void);

#ifdef __cplusplus
}
#endif

#endif