#ifndef __GNSS__KF__MATH__H__
#define __GNSS__KF__MATH__H__

#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PVMODE
#define BIAS_NUM        4
#define DCB_NUM			4

#ifdef PVMODE
#define N_STATE         (7 + BIAS_NUM + DCB_NUM * 2)
#else
#define N_STATE         (10 + BIAS_NUM + DCB_NUM * 2)
#endif

#define N_MAT           (((N_STATE * (N_STATE + 1)) >> 1) + 1)

#define KF_INIT         0x01
#define KF_RESET        0x02
#define KF_RESTART      0x04
#define KF_IDLE         0x08
#define KF_RUN          0x10

#define INO_THRES       30
/* upper part of upper triangular matrix will be stored in vector type
   A = [a11 a12 a13
         0  a22 a23
         0   0  a33]
   is stored as [a11, a12, a13, a22, a23, a33]. About half memory size is saved.
*/

typedef struct
{
    uint8_t      status;                     // KF status flag
    uint8_t      forceReset;                 // KF need force reset in abnormal case
    int16_t     periodTms;                  // KF update period
    double     tor;                    // The time KF update
    double     X[N_STATE];                 // KF state variables
    double     A[N_MAT];                   // KF state transfer matrix, upper triangular matrix
    double     U_plus[N_MAT];              // upper triangular matrix of UD decomposition of P(k) matrix
    double     U_minus[N_MAT];             // upper triangular matrix of UD decomposition of P(k|k-1) matrix
    double     D_plus[N_STATE + 1];        // diagonal matrix of UD decomposition of P(k) matrix
    double     D_minus[N_STATE + 1];       // diagonal matrix of UD decomposition of P(k|k-1) matrix
    double     W1[N_MAT];                  // W1 = A*U, upper triangular matrix
    double     W2[N_MAT];                  // W2 = I(N) when initialization; W = [W1 W2], W is needed in UD KF calculation
    double     Q[N_STATE][N_STATE];        // process noise covariance matrix
} KF_INFO;




int16_t uMatIdx(int16_t i, int16_t j, int16_t n);
void uxuMat(double* U1, double* U2, double* U3, int16_t n);
void uMatxVect(double* U, double* V1, double* V2, int16_t n);
void uMatInit(double* U, int16_t n);
void uMat2Vector(double U1[][N_STATE], double* U2);
void udKfPredict(void);
double  udKFUpdate(double* H, double* deltaX, double R, double ino,double testThres, uint8_t updateType, uint8_t flag,uint8_t testFlag);
void getUdMatDiag(double* U, double* D, double* diag, int16_t n);
void getUdMatP(double* U, double* D, float pMat[N_STATE][N_STATE], int16_t N);

#ifdef __cplusplus
}
#endif


#endif