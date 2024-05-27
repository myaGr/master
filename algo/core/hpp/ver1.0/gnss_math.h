#ifndef __GNSS__MATH__H__
#define __GNSS__MATH__H__

#include <float.h>
#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLT_IS_EQUAL(a, b)     (fabsf((a)-(b)) < FLT_EPSILON)
#define FLT_ISNT_EQUAL(a, b)   (fabsf((a)-(b)) >= FLT_EPSILON)
#define DBL_IS_EQUAL(a, b)     (fabs((a)-(b))  < DBL_EPSILON)
#define DBL_ISNT_EQUAL(a, b)   (fabs((a)-(b))  >= DBL_EPSILON)

void gnssConvEcef2Lla(const double* ecef_in, double* lla_out);
void gnssConvLla2Ecef(double* lla_in, double* ecef_out);
void gnssConvEcef2EnuVel(float* ecefVel, float* enuVel, double* lla);
void gnssConvEnu2EcefVel(float* enuVel, float* ecefVel,  double* lla);
void gnssGetEcef2EnuMatrix(float Ceg[3][3], double* lla);
void gnssGetEnuPmatVar(double* U, double* D, double* lla, double* diagTmp, float* Diag, float* enPosCov, float ecefPosP[6]);
float gnssCalPosDis(const double* lla, double* lla_ref, uint8_t flag);
double gnssClcAvg_DBL(double* A, uint8_t N);
float gnssClcSqrtSum_FLT(const float* A, uint8_t N);
double gnssClcSqrtSum_DBL(double* A, uint8_t N);
float gnssClcSqrtAminusB_FLT(float* A, float* B, uint8_t N);
double gnssClcSqrtAminusB_DBL(double* A, double* B, uint8_t N);
double gnssClcSquareSum_DBL(double* A, uint8_t N);
double  gnssClcVectorMulti(double* A, double* B, uint8_t N);
void gnssEarthRotateCorr(double* satPos, double* satPosCorr, double dt);
int8_t gnss_SignFLT(float m_flt);
uint8_t gnss_QR_Factorize(float HMatrix[][15],uint8_t row, uint8_t colume);
void gnss_Inverse_UpMatrix(float HMatrix[][15], float invR[][15],uint8_t colume);
void gnss_UpMatrix_Square(float invR[][15],float output[15][15],uint8_t colume);
void gnss_Sort_WithIndx(float* a,uint8_t* indx,uint32_t n);
void gnss_Sort_WithIndx_1(float* a,float* b,uint8_t* indx,uint32_t n);
void gnss_Calc_distance(float* a,float* b,uint32_t n);
int16_t SIGN_D(double* p_dbl);
int16_t SIGN_F(float* p_flt);
float gnss_asinf(float f);
void gnss_math_fstd(float* data, uint32_t N, float* mean, float* std);
void gnss_math_dstd(double* data, uint32_t N, double* mean, double* std);
double gnssClcAvg_FLT(float* A, uint8_t N);
uint8_t gnss_median(float* data,uint32_t N,float* out);
uint8_t gnss_median_dbl(double* data,uint32_t N,double* out);
uint8_t gnss_MAD(float* data,float* dataAbs,uint8_t N,float* medianOut);
uint8_t gnss_MAD_DBL(double* data,double* dataAbs,uint8_t N,double* medianOut);
float gnss_lla2_heading(double* lla1, double* lla2);
uint8_t  gnss_kMean(uint32_t totalCnt,float* data, uint8_t* index_cluster,uint8_t flag);
uint8_t  gnss_kMediods(uint32_t totalCnt,float* data, uint8_t* index_cluster);
void gnss_Cluster_Sort(float* dis_avg,uint32_t* cnt_cluster,uint32_t* cluster_indx,uint32_t* goodCnt,uint8_t cluster_N);
void gnss_math_data_smooth(float dataInput, float* dataBack, uint8_t* dataBackCnt,uint8_t maxCnt, float* smoothData);
double gnss_math_min(double *data, int n);
void gnss_Matrix_Mult(float a[3][3], float b[3][3], float c[3][3]);
void gnss_Math_MatrixTrans(const double* a, uint8_t N, uint8_t M, double *b);
void gnss_Math_MatrixDot(const double* a, uint8_t N1, uint8_t M1, const double *b, uint8_t N2, uint8_t M2, double *c);
void gnss_Math_MatrixDot_DF(const double* a, uint8_t N1, uint8_t M1, const float *b, uint8_t N2, uint8_t M2, float *c);
void gnss_Math_Euler2DCM(double* Euler, double* cbn);
#ifdef __cplusplus
}
#endif

#endif
