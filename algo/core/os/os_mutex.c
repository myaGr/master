/**@file        os_mutex.cpp
 * @brief       mutex source file
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
#include "os_mutex.h"
#if defined(_MSC_VER)

#include "windows/mutex_win.c"

#elif defined(__gnu_linux__)

#include "linux/mutex_linux.c"

#else

BOOL mutex_init(mutex_t* mutex, const char* name)
{
  return TRUE;
}

BOOL mutex_destroy(mutex_t* mutex)
{
  return TRUE;
}

BOOL mutex_lock(mutex_t* mutex)
{
  return TRUE;
}

BOOL mutex_unlock(mutex_t* mutex)
{
  return TRUE;
}

BOOL signal_init(signal_t* signal)
{
  return TRUE;
}

BOOL signal_wait(signal_t* signal, mutex_t* mutex, uint32_t tm)
{
  return TRUE;
}

BOOL signal_trigger(signal_t* signal) 
{
  return TRUE;
}

BOOL signal_destroy(signal_t* signal)
{
  return TRUE;
}

#endif
