#ifndef _GNSS_SYS_API_H_
#define _GNSS_SYS_API_H_

#include "mw_alloc.h"
#ifdef _WIN32

    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>

#elif defined(__GNUC__) && defined(__linux__)

    #include <pthread.h>
    #include <semaphore.h>

#elif defined(__STA8100__)

#endif

#include <time.h>
#include <stdio.h>
#include "macro.h"
#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    PROC_EXIT,
    PROC_SUCCESS,
    PROC_FAIL,
    PROC_WRONG_PARAM
};

#ifdef _WIN32
    #define FILEPATHSEP        '\\'
#else
    #define FILEPATHSEP        '/'
#endif

#define FPS            	FILEPATHSEP
#define MAX_PATH_LEN    256

#define DEBUG_STREAM	stdout

/* Logging functions */

#if defined(_WIN32)	// vs2010
    #define SNPRINTF    _snprintf
    #define SNPRINTF_S  _snprintf_s
#else
    #define SNPRINTF  snprintf
#endif

#ifdef _WIN32
#define NEWLINE "\n\r"
#else
#define NEWLINE "\n"
#endif

#define LOG_BUF_LEN 1024

/* Module id defition. */
typedef enum
{
    OBJ_SYS = 0,
    OBJ_API,
    OBJ_RE,
    OBJ_RTCM,
    OBJ_KF,
    OBJ_LS,
    OBJ_SD,
    OBJ_TM,
    OBJ_PE,
    OBJ_QOS,
    OBJ_PCSIM,
    OBJ_DE,
    OBJ_DATA,
    OBJ_AL,
    OBJ_RTK,
    OBJ_COMMON,
    OBJ_ALGO
} Gnss_Module_t;

void gnss_log_print(int level, int line, Gnss_Module_t tag, const char* format, ...);
#define GLOGD(fmt, ...)  gnss_log_print(LOG_DEBUG, __LINE__, MODULE_TAG, fmt, ##__VA_ARGS__)
#define GLOGI(fmt, ...)  gnss_log_print(LOG_INFO, __LINE__, MODULE_TAG, fmt, ##__VA_ARGS__)
#define GLOGW(fmt, ...)  gnss_log_print(LOG_WARNING, __LINE__, MODULE_TAG, fmt, ##__VA_ARGS__)
#define GLOGE(fmt, ...)  gnss_log_print(LOG_ERROR, __LINE__, MODULE_TAG, fmt, ##__VA_ARGS__)
#define SYS_LOGGING(obj, level, fmt, ...) gnss_log_print(level, __LINE__, obj, fmt, ##__VA_ARGS__)

#ifdef FEATURE_USE_FAST_MEMORY
#define Sys_Malloc(n)             OS_MALLOC_FAST(n)
#define Sys_Calloc(n, size)       OS_MALLOC_FAST(n*size)
#else
#define Sys_Malloc(n)             OS_MALLOC(n)
#define Sys_Calloc(n, size)       OS_CALLOC(n,size)
#endif
#define Sys_Free(p)               OS_FREE(p)
#define Sys_Mem_Enable()
#define Sys_Mem_Disable()
#define Sys_Mem_Show()

#ifdef __cplusplus
}
#endif

#endif
