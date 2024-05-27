#ifndef QXIDS_TYPES_H
#define QXIDS_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QXIDS_GGA_MAX_LEN     256U
#define QXIDS_GGA_STR_BUF_LEN 512U
#define QXIDS_GGA_MAX_FIELD   64U /* max number of fields in a record */

#ifndef QXIDS_OK
#define QXIDS_OK (0)
#endif

#ifndef QXIDS_ERROR
#define QXIDS_ERROR (-1)
#endif

#ifndef QXIDS_STATE
#define QXIDS_STATE
/**
 * @brief qxwz ids sdk state enumeration
 */
typedef enum
{
    QXIDS_IDLE,
    QXIDS_INITED,
    QXIDS_STARTED,
} sdk_stat_e;
#endif /* QXIDS_STATE */

/** character */
#ifndef QXIDS_CHAR_T
#define QXIDS_CHAR_T
typedef char qxids_char_t;
#endif

/* 8bits signed integer */
#ifndef QXIDS_INT8_T
#define QXIDS_INT8_T
typedef signed char qxids_int8_t;
#endif

/* 8bits unsigned integer */
#ifndef QXIDS_UINT8_T
#define QXIDS_UINT8_T
typedef unsigned char qxids_uint8_t;
#endif

/* 16bits signed integer */
#ifndef QXIDS_INT16_T
#define QXIDS_INT16_T
typedef signed short qxids_int16_t;
#endif

/* 16bits unsigned integer */
#ifndef QXIDS_UINT16_T
#define QXIDS_UINT16_T
typedef unsigned short qxids_uint16_t;
#endif

/* 32bits signed integer */
#ifndef QXIDS_INT32_t
#define QXIDS_INT32_t
typedef signed int qxids_int32_t;
#endif

/* 32bits unsigned integer */
#ifndef QXIDS_UINT32_T
#define QXIDS_UINT32_T
typedef unsigned int qxids_uint32_t;
#endif

#ifndef QXIDS_LONG32_T
#define QXIDS_LONG32_T
typedef signed long qxids_long32_t;
#endif

#ifndef QXIDS_ULONG32_T
#define QXIDS_ULONG32_T
typedef unsigned long qxids_ulong32_t;
#endif

/* 64bits signed integer */
#ifndef QXIDS_INT64_T
#define QXIDS_INT64_T
typedef signed long long qxids_int64_t;
#endif

/* 64bits unsigned integer */
#ifndef QXIDS_UINT64_T
#define QXIDS_UINT64_T
typedef unsigned long long qxids_uint64_t;
#endif

/* data length */
#ifndef QXIDS_SIZE_T
#define QXIDS_SIZE_T
typedef size_t qxids_size_t;
#endif

/* single precision float number */
#ifndef QXIDS_FLOAT32_T
#define QXIDS_FLOAT32_T
typedef float qxids_float32_t;
#endif

/* double precision float number */
#ifndef QXIDS_FLOAT64_T
#define QXIDS_FLOAT64_T
typedef double qxids_float64_t;
#endif

/* qxids_size_t */
#ifndef QXIDS_SIZE_T
#define QXIDS_SIZE_T
typedef size_t qxids_size_t;
#endif

/* qxids_void_t */
#ifndef QXIDS_VOID_T
#define QXIDS_VOID_T
typedef void qxids_void_t;
#endif

/* boolean representation */
#ifndef QXIDS_BOOL_T
#define QXIDS_BOOL_T
typedef enum
{
    /* FALSE value */
    QXIDS_FALSE,
    /* TRUE value */
    QXIDS_TRUE
} qxids_bool_t;
#endif /* QXIDS_BOOL_T */

/* NULL */
#ifndef QXIDS_NULL
#define QXIDS_NULL (qxids_void_t*)0
#endif

/* gtime_t */
#ifndef QXIDS_GTIME_T
#define QXIDS_GTIME_T
typedef struct
{                         /* time struct */
    time_t          time; /* time (s) expressed by standard time_t */
    qxids_float64_t sec;  /* fraction of second under 1 s */
} qxids_gtime_t;
#endif  // QXIDS_GTIME_T

#ifdef __cplusplus
}
#endif
#endif /* QXIDS_TYPES_H */
