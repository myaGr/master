/**
 ***************************************************************************
 * Copyright (C), 2016 ASG Corporation.
 *
 * \file	ag_base.h
 * \version 
 * \author  caizhijie
 * \date    2017/09/05 14:28
 * \warning 
 * Email    caizhijie_hit@163.com
 ***************************************************************************
 * \brief   
 * \attention
 *
 */

#ifndef __AG_BASE_H__
#define __AG_BASE_H__

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include "macro.h"

#if defined(__cplusplus) || defined(c_plusplus)
# define BEGIN_DECL     extern "C" {
# define END_DECL       }
# define EXTERNC        extern "C" 
#else
# define BEGIN_DECL
# define END_DECL
# define EXTERNC
#endif

#if defined(_WIN32)
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

#define AG_OK	(1)
#define AG_ERR	(0)
#define AG_SUCC (1)
#define AG_FAIL (0)


#ifdef ENABLE_ASSERT
#   define _OS_ASSERT(exp) Gnss_Assert((uint8_t const*) #exp, (uint8_t const*) __FILE__, NULL, __LINE__)
#   define GNSS_ASSERT(exp) \
    do { \
    if(!(exp)) \
    { \
    _OS_ASSERT(exp); \
    } \
    } while(0);
#else
#   define GNSS_ASSERT(exp)
#endif

#if defined( _WIN32)
#undef OS_UCOS

#define OS_ERR uint32_t
#define CPU_SR_ALLOC
#define OS_CRITICAL_ENTER
#define OS_CRITICAL_EXIT
#define OSTaskDel

#ifndef DLL_EXPORTS
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __declspec(dllimport)
#endif


#elif defined(GNUC_MACRO)
#undef OS_UCOS

#define OS_ERR uint32_t
#define CPU_SR_ALLOC
#define OS_CRITICAL_ENTER
#define OS_CRITICAL_EXIT
#define OSTaskDel
#define DLLEXPORT

#elif defined (OS_UCOS)

#define DLLEXPORT

#endif

/* safe free memory */
#define SAFE_FREE(x) {if(x!=NULL){free(x);x=NULL;}}

#ifndef TRUE
#define TRUE	(1)
#endif

#ifndef FALSE
#define FALSE	(0)
#endif

#ifndef SUCCESS
#define SUCCESS (0)
#endif

#ifndef FAIL
#define FAIL    (-1)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)     (sizeof(x)/sizeof(x[0]))
#endif

#define FLOAT_EPS (1e-6)
#define DOUBLE_EPS (1e-15)
#define FLOAT_IS_ZERO(x)  (fabs(x)<FLOAT_EPS?1:0) 
#define DOUBLE_IS_ZERO(x) (fabs(x)<DOUBLE_EPS?1:0) 
#define FLOAT_COMPARE(x,y)  (((x)-(y))>FLOAT_EPS?1:-1) 
#define DOUBLE_COMPARE(x,y) (((x)-(y))>DOUBLE_EPS?1:-1) 

#endif
