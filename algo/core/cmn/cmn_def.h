/**@file        cmn_def.h
 * @brief
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/04  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __CMN_DEF_H__
#define __CMN_DEF_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef ENABLE_DELOS_PRINTF
#include "ks_printf.h"
#endif

#if defined(__cplusplus) || defined(c_plusplus)
  #define BEGIN_DECL     extern "C" {
  #define END_DECL       }
  #define EXTERNC        extern "C" 
#else
  #define BEGIN_DECL
  #define END_DECL
  #define EXTERNC
#endif


BEGIN_DECL

#ifndef TRUE
#define TRUE  ((int32_t)1)
#endif

#ifndef FALSE
#define FALSE ((int32_t)0)
#endif

#ifndef BOOL
#define BOOL  int32_t
#endif

#ifndef DBL_EPSILON
#define DBL_EPSILON   2.2204460492503131e-016 // smallest such that 1.0+DBL_EPSILON != 1.0
#endif
#define DBL_EQU(x, y) (fabs((x)-(y))<(2 * DBL_EPSILON))

#ifndef FLT_EPSILON
#define FLT_EPSILON   1.192092896e-07F        // smallest such that 1.0+FLT_EPSILON != 1.0
#endif
#define FLT_EQU(x, y) (fabs((x)-(y))<(2 * FLT_EPSILON))

/* use tlsf memory alloc algorithm */
#define FEATURE_MEM_USE_ALLOC    (0)
#define FEATURE_MEM_USE_TLSF     (1)
#define FEATURE_MEM_USE_EXT_API  (2)
#define FEATURE_MEM_USE_ID       (FEATURE_MEM_USE_TLSF)

/* */
#define FEATURE_PVT_V2

#define LOC_VERSION_MAJOR 1
#define LOC_VERSION_MINOR 0

#define FABS_ZEROS          ((double)1.0e-15)

END_DECL

#endif
