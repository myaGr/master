/*------------------------------------------------------------------------------
* lambda.c : integer ambiguity resolution
*
*          Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
*         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
*         1995
*     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
*         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/13 1.0 new
*           2015/05/31 1.1 add api lambda_reduction(), lambda_search()
*-----------------------------------------------------------------------------*/
#include "mlambda.h"

/* constants/macros ----------------------------------------------------------*/
static double RTK_ADOP = -1.0;

extern double rtk_adop_get(void)
{
  return RTK_ADOP;
}
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int rtk_LD(int n, const double* Q, double* L, double* D)
{
  int i, j, k, info = 0;
  double a;
  double* A = OS_MALLOC_FAST(sizeof(double) * n * n);;
  if (NULL == A) {
    return -1;
  }

  memcpy(A, Q, sizeof(double) * n * n);
  for (i = n - 1; i >= 0; i--) {
    if ((D[i] = A[i + i * n]) <= 0.0) { info = -1; break; }
    a = sqrt(D[i]);
    for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
    for (j = 0; j <= i - 1; j++) for (k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
    for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
  }
  OS_FREE(A);
  if (info) {
    LOGW(TAG_HPP, "LD factorization error\n");
  }
  return info;
}
/* integer gauss transformation ----------------------------------------------*/
static void rtk_gauss(int n, double* L, double* Z, int i, int j)
{
  int k, mu;

  if ((mu = (int)ROUND(L[i + j * n])) != 0) {
    for (k = i; k < n; k++) L[k + n * j] -= (double)mu * L[k + i * n];
    for (k = 0; k < n; k++) Z[k + n * j] -= (double)mu * Z[k + i * n];
  }
}
/* permutations --------------------------------------------------------------*/
static void rtk_perm(int n, double* L, double* D, int j, double del, double* Z)
{
  int k;
  double eta, lam, a0, a1;

  eta = D[j] / del;
  lam = D[j + 1] * L[j + 1 + j * n] / del;
  D[j] = eta * D[j + 1]; D[j + 1] = del;
  for (k = 0; k <= j - 1; k++) {
    a0 = L[j + k * n]; a1 = L[j + 1 + k * n];
    L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
    L[j + 1 + k * n] = eta * a0 + lam * a1;
  }
  L[j + 1 + j * n] = lam;
  for (k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
  for (k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void rtk_reduction(int n, double* L, double* D, double* Z)
{
  int i, j, k;
  double del;

  j = n - 2; k = n - 2;
  while (j >= 0) {
    if (j <= k) for (i = j + 1; i < n; i++) rtk_gauss(n, L, Z, i, j);
    del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
    if (del + 1E-6 < D[j + 1]) { /* compared considering numerical error */
      rtk_perm(n, L, D, j, del, Z);
      k = j; j = n - 2;
    }
    else j--;
  }
}
/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
static int rtk_search(int n, int m, const double* L, const double* D,
  const double* zs, double* zn, double* s)
{
  int i, j, k, c, nn = 0, imax = 0;
  double newdist, maxdist = 1E99, y;
  double* S = OS_MALLOC_FAST(n * n * sizeof(double));
  double* dist = OS_MALLOC_FAST(n * sizeof(double));
  double* zb = OS_MALLOC_FAST(n * sizeof(double));
  double* z = OS_MALLOC_FAST(n * sizeof(double));
  double* step = OS_MALLOC_FAST(n * sizeof(double));

  if (NULL == S || NULL == dist || NULL == zb || NULL == z || NULL == step) {
    if (NULL != S)
    {
      OS_FREE(S);
      S = NULL;
    }
    if (NULL != dist)
    {
      OS_FREE(dist);
      dist = NULL;
    }
    if (NULL != zb)
    {
      OS_FREE(zb);
      zb = NULL;
    }
    if (NULL != z)
    {
      OS_FREE(z);
      z = NULL;
    }
    if (NULL != step)
    {
      OS_FREE(step);
      step = NULL;
    }
    return -1;
  }

  k = n - 1; dist[k] = 0.0;
  zb[k] = zs[k];
  z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
  for (c = 0; c < LOOPMAX; c++) {
    newdist = dist[k] + y * y / D[k];
    if (newdist < maxdist) {
      if (k != 0) {
        dist[--k] = newdist;
        for (i = 0; i <= k; i++)
          S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
        zb[k] = zs[k] + S[k + k * n];
        z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
      }
      else {
        if (nn < m) {
          if (nn == 0 || newdist > s[imax]) imax = nn;
          for (i = 0; i < n; i++) zn[i + nn * n] = z[i];
          s[nn++] = newdist;
        }
        else {
          if (newdist < s[imax]) {
            for (i = 0; i < n; i++) zn[i + imax * n] = z[i];
            s[imax] = newdist;
            for (i = imax = 0; i < m; i++) if (s[imax] < s[i]) imax = i;
          }
          maxdist = s[imax];
        }
        z[0] += step[0]; y = zb[0] - z[0]; step[0] = -step[0] - SGN(step[0]);
      }
    }
    else {
      if (k == n - 1) break;
      else {
        k++;
        z[k] += step[k]; y = zb[k] - z[k]; step[k] = -step[k] - SGN(step[k]);
      }
    }
  }
  for (i = 0; i < m - 1; i++) { /* sort by s */
    for (j = i + 1; j < m; j++) {
      if (s[i] < s[j]) continue;
      SWAP(s[i], s[j]);
      for (k = 0; k < n; k++) SWAP(zn[k + i * n], zn[k + j * n]);
    }
  }

  OS_FREE(S);
  S = NULL;
  OS_FREE(dist);
  dist = NULL;
  OS_FREE(zb);
  zb = NULL;
  OS_FREE(z);
  z = NULL;
  OS_FREE(step);
  step = NULL;

  if (c >= LOOPMAX) {
    LOGW(TAG_HPP, "search loop count=%d overflow\n", c);
    return -1;
  }
  return 0;
}
/**
 * @brief solve linear equation
 */
static int8_t rtk_solve(matrix_t* A, matrix_t* Y, double* X)
{
  matrix_t* B = NULL;
  matrix_t* x = NULL;
  int8_t info = 1;

  B = matrix_new(A->row, A->col);
  x = matrix_new_from_list(A->col, X);
  if (NULL == B || NULL == x)
  {
    if (NULL != B)
    {
      matrix_free(&B);
      B = NULL;
    }
    if (NULL != x)
    {
      matrix_free(&x);
      x = NULL;
    }
    return -1;
  }

  if (matrix_inverse(A, B))
  {
    info = 0;
    matrix_mul(B, Y, x);
    memcpy(X, x->data, x->col * x->row * sizeof(double));
  }
  matrix_free(&B);
  B = NULL;
  matrix_free(&x);
  x = NULL;
  return info;
}
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
  double* s)
{
  int i, info;
  matrix_t* L = NULL;
  matrix_t* D = NULL;
  matrix_t* Z = NULL;
  matrix_t* z = NULL;
  matrix_t* E = NULL;
  matrix_t* E1 = NULL;
  matrix_t* A = NULL;

  if (n <= 0 || m <= 0) return -1;

  L = matrix_new(n, n);
  D = matrix_new(n, 1);
  Z = matrix_ones(n);
  z = matrix_new(n, 1);
  E = matrix_new(n, m);
  E1 = matrix_new(n, 1);
  A = matrix_new_from_list(n, a);

  if (NULL == L || NULL == D || NULL == Z || NULL == z || NULL == E || NULL == A || NULL == E1)
  {
    if (NULL != L)
    {
      matrix_free(&L);
      L = NULL;
    }
    if (NULL != D)
    {
      matrix_free(&D);
      D = NULL;
    }
    if (NULL != Z)
    {
      matrix_free(&Z);
      Z = NULL;
    }
    if (NULL != z)
    {
      matrix_free(&z);
      z = NULL;
    }
    if (NULL != E)
    {
      matrix_free(&E);
      E = NULL;
    }
    if (NULL != E1)
    {
      matrix_free(&E1);
      E1 = NULL;
    }
    if (NULL != A)
    {
      matrix_free(&A);
      A = NULL;
    }
    return -1;
  }

  /* LD factorization */
  if (!(info = rtk_LD(n, Q, L->data, D->data))) {

    double adop = 1.0;
    for (i = 0; i < n; i++) adop *= D->data[i];
    if (adop > 0.0) RTK_ADOP = 100.0 * pow(adop, 1.0 / (2.0 * n));

    /* lambda reduction */
    rtk_reduction(n, L->data, D->data, Z->data);

    matrix_mul(Z, A, z); /* z=Z'*a */

        /* mlambda search */
    if (!(info = rtk_search(n, m, L->data, D->data, z->data, E->data, s)))
    {
      memcpy(E1->data, E->data, n * sizeof(double));
      info = rtk_solve(Z, E1, F); /* F=Z'\E */
      if (0 == info)
      {
        memcpy(E1->data, &E->data[n], n * sizeof(double));
        info = rtk_solve(Z, E1, &F[n]); /* F=Z'\E */
      }
    }
  }

  matrix_free(&L);
  L = NULL;
  matrix_free(&D);
  D = NULL;
  matrix_free(&Z);
  Z = NULL;
  matrix_free(&z);
  z = NULL;
  matrix_free(&E);
  E = NULL;
  matrix_free(&E1);
  E1 = NULL;
  matrix_free(&A);
  A = NULL;
  return info;
}


