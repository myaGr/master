/**@file        lambda.h
 * @brief       integer ambiguity resolution for RTK
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/03/06  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __MLAMBDA_H__
#define __MLAMBDA_H__

#include "cmn_def.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "cmn_utils.h"
BEGIN_DECL

#define LOOPMAX     10000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

/* get adop ----------------------------------------------------------------*/
extern double rtk_adop_get(void);

#if 0
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int rtk_LD(int n, const double* Q, double* L, double* D);

/* integer gauss transformation ----------------------------------------------*/
static void rtk_gauss(int n, double* L, double* Z, int i, int j);

/* permutations --------------------------------------------------------------*/
static void rtk_perm(int n, double* L, double* D, int j, double del, double* Z);

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void rtk_reduction(int n, double* L, double* D, double* Z);

/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
static int rtk_search(int n, int m, const double* L, const double* D,
  const double* zs, double* zn, double* s);
#endif

/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
extern int rtk_lambda(int n, int m, const double* a, const double* Q, double* F,
  double* s);

/* lambda reduction ------------------------------------------------------------
* reduction by lambda (ref [1]) for integer least square
* args   : int    n      I  number of float parameters
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *Z     O  lambda reduction matrix (n x n)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
extern int lambda_reduction(int n, const double* Q, double* Z);

/* mlambda search --------------------------------------------------------------
* search by  mlambda (ref [2]) for integer least square
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
extern int lambda_search(int n, int m, const double* a, const double* Q,
  double* F, double* s);

END_DECL
#endif