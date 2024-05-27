/************************************************************
* Copyrights(C) 
* All rights Reserved
* 文件名称：gnss_kf_math.c
* 文件描述：Kalman filter的核心模块
* 版本号：1.0.0
* 作者：
* 日期：08/29
************************************************************/
#include "gnss_kf_math.h"
#include "gnss_kf.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "gnss_sys_api.h"

/* Define the KF control block */
KF_INFO     kfInfo;

/**********************************************************************
* Function Name:    uMatIdx
*
* Description:
*    get the index of upper trianular matrix U(i,j) that stored in vector type
*
* Input:
*     i: row index of matrix
*     j: column index of matrix
*     N:  row and column length of matrix
*
* Return:
*     index of upper triangular that stored in vector type
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int16_t uMatIdx(int16_t i, int16_t j, int16_t N)
{
    int16_t i1, i2;
    i1 = i - 1;
    i2 = i - 2;

    return i1 * N - ((i1 * i2) >> 1) + j - i + 1;
}


/**********************************************************************
* Function Name:    uxuMat
*
* Description:
*    upper triangular matrix multiply upper triangular matrix
*
* Input:
*     U1: upper triangular matrix
*     U2: upper triangular matrix
*     U3: U3 = U1*U2
*     N:  row and column length of matrix
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void uxuMat(double* U1, double* U2, double* U3, int16_t N)
{
    int16_t i, j, k, m, idx1, idx2;
    double tmp;

    m = 1;

    for (i = 1; i <= N; i++)
    {
        for (j = i; j <= N; j++)
        {
            tmp = 0;

            for (k = i; k <= j; k++)
            {
                idx1 = uMatIdx(i, k, N);
                idx2 = uMatIdx(k, j, N);
                tmp += U1[idx1] * U2[idx2];
            }

            U3[m++] = tmp;
        }
    }
}

/**********************************************************************
* Function Name:    uMatInit
*
* Description:
*    initialize the diagonal value of upper triangular matrix to 1
*
* Input:
*     U: upper triangular matrix
*     N:  row and column length of matrix
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void uMatInit(double* U, int16_t N)
{
    int16_t i, idx;

    for (i = 1; i <= N; i++)
    {
        idx = uMatIdx(i, i, N);
        U[idx] = 1;
    }
}

/**********************************************************************
* Function Name:    uMat2Vector
*
* Description:
*    store upper part of U1[N][N] to U2[N*(N+1)/2]
*
* Input:
*     U1: upper triangular matrix in 2 dimension
*     U2: upper triangular matrix in vector type
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void uMat2Vector(double U1[][N_STATE], double* U2)
{
    int16_t i, j, idx, N;
    N = N_STATE;

    for (i = 1; i <= N; i++)
    {
        for (j = i; j <= N; j++)
        {
            idx = uMatIdx(i, j, N);
            U2[idx] = U1[i - 1][j - 1];
        }
    }
}


/**********************************************************************
* Function Name:    uMatxVect
*
* Description:
*    upper triangular matrix multiply vector, V2 = U*V1;
*
* Input:
*    U: upper triangular matrix, N*N
*    V1: vector array1, N*1, idx start from 0
*    V2: vector array2x, N*1, idx start from 0
*    N:  row and column length of matrix
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void uMatxVect(double* U, double* V1, double* V2, int16_t N)
{
    int16_t i, j, idx;
    double tmp;

    for (i = 1; i <= N; i++)
    {
        idx = uMatIdx(i, i, N);
        tmp = 0;

        for (j = i; j <= N; j++)
        {
            tmp += U[idx] * V1[j - 1];
            idx++;
        }

        V2[i - 1] = tmp;
    }
}

/**********************************************************************
* Function Name:    getUdMatDiag
*
* Description:
*    get the diagonal components of U*D*D'
*
* Input:
*    U:     upper triangular matrix of UD decomposition, N*N
*    D:     diagonal matrix of UD decomposition, 1*N
*    diag:  diagonal components, 1*N
*    N:     row and column length of matrix
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void getUdMatDiag(double* U, double* D, double* diag, int16_t N)
{
    int16_t i, k, idx;
    double tmp;

    for (i = 1; i <= N; i++)
    {
        tmp = 0;

        idx = uMatIdx(i, i, N);

        for (k = i; k <= N; k++)
        {
            tmp += U[idx] * D[k] * U[idx];
            idx = idx + 1;
        }

        diag[i - 1] = tmp;
    }
}
/**********************************************************************
* Function Name:    getUdMatP
*
* Description:
*    get P by  U*D*D'
*    
* Input:
*    U:     upper triangular matrix of UD decomposition, N*N
*    D:     diagonal matrix of UD decomposition, 1*N
*    N:     row and column length of matrix
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void getUdMatP(double* U, double* D, float pMat[N_STATE][N_STATE], int16_t N)
{
  int16_t     i, j, k, m;
	int16_t     /*index,*/ idx1, idx2;
	double     tmp, tmp1, tmp2;
	double     UD[N_MAT] = { 0.0 }/*, UDU[3][3]*/;


	m = 1;

	for (i = 1; i <= N; i++)
	{
		for (j = i; j <= N; j++)
		{
			//tmp = 0;

			idx1 = uMatIdx(i, j, N);
			tmp = U[idx1] * D[j];
			UD[m++] = tmp;
		}
	}

	for (i = 1; i <= N; i++)
	{
		for (j = 1; j <= N; j++)
		{
			tmp = 0;

			for (k = 1; k <= N; k++)
			{
				if (k < i)
				{
					tmp1 = 0;
				}
				else
				{
					idx1 = uMatIdx(i, k, N);
					tmp1 = UD[idx1];
				}
				if (j > k)
				{
					tmp2 = 0;
				}
				else
				{
					idx2 = uMatIdx(j, k, N);
					tmp2 = U[idx2];
				}
				tmp += tmp1 * tmp2;
			}
			pMat[i - 1][j - 1] = (float)tmp;
		}
	}
}

/**********************************************************************
* Function Name:  uduDecomp
*
* Description:
*       Break up asymmetry matrix to matrix U and D.
*
* Input:
*        p: Be decomposed matrix.
n: dimention of matrix p

* Return:
*      None

* Dependency
*      None
*
* Author: 
* Date: 03/13
**********************************************************************/
void uduDecomp(double* p, uint8_t n)

{
	const double EPS = 1.0e-8;
	uint8_t i, j, k;
	double amax, alpha, beta;

	double(*LP)[N_STATE];
	LP = (double(*)[N_STATE])(p);


	// Search for maximum of diagonal elements and scale down by 1.0e+8
	// for threshold for setting alpha to zero.
	amax = 0.0;

	for (i = 0; i <= n - 1; i++)
		if (amax < LP[i][i])
		{
			amax = LP[i][i];
		}

	amax = amax * EPS;

	for (j = n; j >= 2; j--)
	{
		if (LP[j - 1][j - 1] > amax)
		{
			alpha = (double)1.0 / LP[j - 1][j - 1];

			for (k = 1; k <= j - 1; k++)
			{
				beta = LP[k - 1][j - 1];
				LP[k - 1][j - 1] = alpha * beta;

				for (i = 1; i <= k; i++)
				{
					LP[i - 1][k - 1] -= beta * LP[i - 1][j - 1];
				}
			}    // k in 1..j-1
		}
		else
			for (k = 1; k <= j - 1; k++)
			{
				LP[k - 1][j - 1] = 0.0;
			}
	}    // j in reverse 2..n
}
/**********************************************************************
* Function Name:    udKfPredict
*
* Description:
*    predict X(k|k-1) and P(k|k-1)
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void udKfPredict(void)
{
    int16_t i, j, k, m, idx, idx1, N, idx_ij, idx_jj;
    double tmp, W1jk, WD;

    N = N_STATE;
    /* KF state prediction, X(k|k-1) = A*X(k-1) */
    uMatxVect(kfInfo.A, kfInfo.X, kfInfo.X, N_STATE);

    //memset(kfInfo.W1, 0, sizeof(double)*N_MAT);
    memset(kfInfo.W2, 0, sizeof(double)*N_MAT);
    uMatInit(kfInfo.W2, N);
    uxuMat(kfInfo.A, kfInfo.U_plus, kfInfo.W1, N);

    /* P(k|k-1) = A*P(k-1)*A' + Q, calculate UD of P(k|k-1) */
    for (j = N; j >= 1; j--)
    {
        /* calculate D */
        tmp = 0;
        idx_jj = uMatIdx(j, j, N);
        idx = idx1 = idx_jj;

        for (k = j; k <= N; k++)
        {
            W1jk = kfInfo.W1[idx];
            WD = W1jk * kfInfo.D_plus[k];
            tmp += WD * W1jk;
            idx++;
        }

        for (k = j; k <= N; k++)
        {
            WD = 0;
            idx = idx_jj;

            for (m = j; m <= N; m++)
            {
                WD += kfInfo.W2[idx] * kfInfo.Q[m - 1][k - 1];
                idx++;
            }

            tmp += WD * kfInfo.W2[idx1];
            idx1++;
        }

        kfInfo.D_minus[j] = tmp;

        /* calculate U */
        for (i = 1; i <= j - 1; i++)
        {
            tmp = 0;
            idx_ij = uMatIdx(i, j, N);
            idx = idx_ij;
            idx1 = idx_jj;

            for (k = j; k <= N; k++)
            {
                WD = kfInfo.W1[idx] * kfInfo.D_plus[k];
                tmp += WD * kfInfo.W1[idx1];
                idx++;
                idx1++;
            }

            idx1 = idx_jj;

            for (k = j; k <= N; k++)
            {
                WD = 0;
                idx = uMatIdx(i, i, N);

                for (m = i; m <= N; m++)
                {
                    WD += kfInfo.W2[idx] * kfInfo.Q[m - 1][k - 1];
                    idx++;
                }

                tmp += WD * kfInfo.W2[idx1];
                idx1++;
            }

            idx = idx_ij;

            if (fabs(kfInfo.D_minus[j]) > 1e-10)
            {
                kfInfo.U_minus[idx] = tmp / kfInfo.D_minus[j];
            }
            else
            {
                kfInfo.U_minus[idx] = tmp * 1e10;
            }

            idx = idx_jj;
            idx1 = idx_ij;

            for (k = j; k <= N; k++)
            {
                kfInfo.W1[idx1] -= kfInfo.U_minus[idx_ij] * kfInfo.W1[idx];
                kfInfo.W2[idx1] -= kfInfo.U_minus[idx_ij] * kfInfo.W2[idx];
                idx++;
                idx1++;
            }
        }
    }//end for (j = N; j >= 1; j--)
}

/**********************************************************************
* Function Name:    udKFUpdate
*
* Description:
*    UD KF scalar update
*
* Input:
*     H:            measurement matrix, 1*N
*     R:            measurement noise variance, 1*1
*     ino:          innovation, 1*1
*     delatX:       state correction
*     updateType:   MEAS_PR_UPDATE, MEAS_DR_UPDATE, COAST_UPDATE
*     flag:         1 -- save U,D results
*                   0 -- don't save U,D results
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double udKFUpdate(double* H, double* deltaX, double R, double ino,double testThres, uint8_t updateType, uint8_t flag,uint8_t testFlag)
{
    int16_t i, j, k, idx, N;
    double test = 0.0;
    double f[N_STATE + 1], v[N_STATE + 1], b[N_STATE + 1], p;
    double a0, a1;

    N = N_STATE;

    /* Gain = P(k|k-1)*H'/[H*P(k|k-1)*H' + R], P(k) = (I - Gain*H)*P(k|k-1), calculate UD of P(k) */
    for (i = 1; i <= N; i++)
    {
        f[i] = 0;

        for (k = 1; k <= i; k++)
        {
            idx = uMatIdx(k, i, N);
            f[i] += H[k - 1] * kfInfo.U_minus[idx];
        }
    }

    for (i = 1; i <= N; i++)
    {
        v[i] = kfInfo.D_minus[i] * f[i];
    }

    a0 = R;

    for (i = 1; i <= N; i++)
    {
        a1 = a0 + f[i] * v[i];
        kfInfo.D_plus[i] = kfInfo.D_minus[i] * a0 / a1;
        b[i] = v[i];
        p = -f[i] / a0;
        a0 = a1;

        for (j = 1; j <= i - 1; j++)
        {
            idx = uMatIdx(j, i, N);
            kfInfo.U_plus[idx] = kfInfo.U_minus[idx] + b[j] * p;
            b[j] += kfInfo.U_minus[idx] * v[i];
        }
    }

    // b[] is gain
    for (i = 1; i <= N; i++)
    {
        b[i] = b[i] / a1;
    }

    // innovation error variance check
	test = (double)(fabs(ino) / sqrt(a1));
    if (flag)
    {
        if (updateType == MEAS_PR_UPDATE)
        {
            if (test > testThres && testFlag == TRUE)
            {
                SYS_LOGGING(OBJ_KF,LOG_INFO,"PR cannot not pass test check: %f", test);
                return test;
            }
        }

        if (updateType == MEAS_DR_UPDATE)
        {
            if (test > testThres && testFlag == TRUE)
            {
                SYS_LOGGING(OBJ_KF,LOG_INFO,"DR cannot not pass test check: %f", test);
                return test;
            }
        }
    }

    // calculate update
    for (i = 1; i <= N; i++)
    {
        deltaX[i - 1] += (double)(b[i] * ino);
    }

    if (flag)
    {
        // prepare for next update
        memcpy(kfInfo.D_minus, kfInfo.D_plus, sizeof(double) * (N + 1));
        memcpy(kfInfo.U_minus, kfInfo.U_plus, sizeof(double)*N_MAT);
    }

    return test;
}