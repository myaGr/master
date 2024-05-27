#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gnss.h"
#include "gnss_types.h"
#include "gnss_math.h"
#include "gnss_kf_math.h"
#include "gnss_sys_api.h"

#define EPSILON_ERROR_GUARD
#define EPSILON_ERROR_TOLERANCE     5

//Default based on ieee standard
float f_epsilon = (float)1.19209E-007;
double d_epsilon = (double)2.22045E-016;

/**********************************************************************
The following will return -1 if *p_flt is negative, 0 if it is zero,
and 1 if it is positive.
**********************************************************************/
int16_t SIGN_F(float* p_flt)
{
  if (*p_flt < 0)
  {
    return -1;
  }

  if (*p_flt > 0)
  {
    return 1;
  }

  return 0;
}
/**********************************************************************
The following will return -1 if *p_dbl is negative, 0 if it is zero,
and 1 if it is positive.
**********************************************************************/
int16_t SIGN_D(double* p_dbl)
{
  if (*p_dbl < (double)0)
  {
    return -1;
  }

  if (*p_dbl > (double)0)
  {
    return 1;
  }

  return 0;
}

/**********************************************************************
* Function Name:    gnssConvEcef2Lla
*
* Description:
*    convert ecef position to lla position
*
* Input:
*     ecef_in: pointer to ECEF position
*     lla_out: pointer to lla position
*              0: latitude, 1: longitude, 2: altitude
* Return:
*
*
* Dependency
*      None
*
* Author:
**********************************************************************/
void gnssConvEcef2Lla(const double* ecef_in, double* lla_out)
{
  double    r;           /* Radius dist to the point in the  X-Y Plane*/
  double    Theta;       /* Approximate Latitude of point */
  double    SinTheta;    /* Sine of Theta */
  double    Sin2Theta;   /* Square of the Sine of Theta */
  double    Sin3Theta;  /* Cube of the Sine of Theta */
  double    CosTheta;    /* Cosine of Theta */
  double    Cos3Theta;  /* Cube of the Cosine of Theta */
  double    SinLat;
  double    Sin2Lat; /* Sine of Latitude */
  double    CosLat;      /* Cosine of the Latitude */
  double   lla[3], ecef[3];

  ecef[0] = ecef_in[0];
  ecef[1] = ecef_in[1];
  ecef[2] = ecef_in[2];
  /******************************************************************/
  /* compute the radial distance of the point in the X-Y-plane. */
  r = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
  /* compute the approximate latitude */
  Theta = atan((A_WGS84 * ecef[2]) / (r * B_WGS84));
  /* Take the cube of the sine of theta */
  SinTheta = sin(Theta);
  Sin2Theta = SinTheta * SinTheta;
  Sin3Theta = SinTheta * Sin2Theta;
  /* Take the cube of the cosine of theta */
  CosTheta = sqrt((double)1.0 - Sin2Theta);    /* CosTheta = cos(Theta); */
  Cos3Theta = CosTheta * CosTheta * CosTheta;
  /* Calculate the Latitude */
  lla[0] = atan((ecef[2] + EARTH_TEMP_2 * B_WGS84 * Sin3Theta) /
    (r - EARTH_TEMP_1 * A_WGS84 * Cos3Theta));
  /* Calculate the Longitude */
  lla[1] = atan2(ecef[1], ecef[0]);
  SinLat = sin(lla[0]);
  Sin2Lat = SinLat * SinLat;
  /* quickly compute the cosine of the latitude */
  CosLat = sqrt((double)1.0 - Sin2Lat);

  /* Calculate the Height checking for a polar exception */
  if (r > (double)1.0)
  {
    lla[2] = (r / CosLat) - (A_WGS84 / sqrt((double)1.0 - EARTH_TEMP_1 * Sin2Lat));
  }
  else
  {
    lla[2] = fabs(ecef[2]) - B_WGS84;
  }

  lla_out[0] = lla[0];
  lla_out[1] = lla[1];
  lla_out[2] = lla[2];

  return;
} /* conv_ecef_lla() */
/**********************************************************************
* Function Name:    gnssConvLla2Ecef
*
* Description:
*       1. convert lla position to ecef position in float format;
*
* Input:    lla position
*
* Output:   ecef position
*
* Dependency
*      None
*
* Author:
**********************************************************************/
void gnssConvLla2Ecef(double* lla_in, double* ecef_out)
{
  double  sinlat, coslat;
  double  sinlon, coslon;
  double  Rn, tmp;
  double   lla[3], ecef[3];

  lla[0] = (double)lla_in[0];
  lla[1] = (double)lla_in[1];
  lla[2] = (double)lla_in[2];

  sinlat = sin(lla[0]);
  coslat = cos(lla[0]);
  sinlon = sin(lla[1]);
  coslon = cos(lla[1]);
  Rn = A_WGS84 / sqrt((double)1.0 - (E2_WGS84 * sinlat * sinlat));
  tmp = (Rn + lla[2]) * coslat;
  ecef[0] = tmp * coslon;
  ecef[1] = tmp * sinlon;
  ecef[2] = (Rn * ONE_MIN_E2 + lla[2]) * sinlat;

  ecef_out[0] = ecef[0];
  ecef_out[1] = ecef[1];
  ecef_out[2] = ecef[2];
}
/**********************************************************************
* Function Name:    gnssCalPosDis
*
* Description:
*       1. calculate two position distance;
*
* Input:    lla position
*           flag : 0-3d, 1--2d
*
* Output:   two position distance
*
* Dependency
*      None
*
* Author:
**********************************************************************/
float gnssCalPosDis(const double* lla, double* lla_ref, uint8_t flag)
{
  float      posDis;
  double      diff_lon;
  double      posDis_e, posDis_n, alt_diff;

  diff_lon = lla[1] - lla_ref[1];

  if (diff_lon > PI)
  {
    diff_lon = diff_lon - PI2;
  }
  else if (diff_lon < -PI)
  {
    diff_lon = diff_lon + PI2;
  }

  posDis_e = A_WGS84 * cos(lla_ref[0]) * diff_lon;
  posDis_n = A_WGS84 * (lla[0] - lla_ref[0]);
  alt_diff = lla[2] - lla_ref[2];

  if (!flag)
  {
    posDis = (float)sqrt(posDis_e * posDis_e + posDis_n * posDis_n + alt_diff * alt_diff);
  }
  else
  {
    posDis = (float)sqrt(posDis_e * posDis_e + posDis_n * posDis_n);
  }

  return posDis;
}
/**********************************************************************
* Function Name:    gnssGetEcef2EnuMatrix
*
* Description:
*    get the convert matrix from ECEF to ENU
*
* Input:
*     Ceg:     pointer to the convert matrix
*     lla_out: pointer to lla position
*              0: latitude, 1: longitude, 2: altitude
* Return:
*
*
* Dependency
*      None
*
* Author:
**********************************************************************/
void gnssGetEcef2EnuMatrix(float Ceg[3][3], double* lla)
{
  float cosLat, cosLon, sinLat, sinLon;

  cosLat = (float)cos(lla[0]);
  sinLat = (float)sin(lla[0]);
  cosLon = (float)cos(lla[1]);
  sinLon = (float)sin(lla[1]);

  Ceg[0][0] = -sinLon;            //E
  Ceg[0][1] = cosLon;
  Ceg[0][2] = 0;

  Ceg[1][0] = -sinLat * cosLon;   //N
  Ceg[1][1] = -sinLat * sinLon;
  Ceg[1][2] = cosLat;

  Ceg[2][0] = cosLat * cosLon;    //U
  Ceg[2][1] = cosLat * sinLon;
  Ceg[2][2] = sinLat;
}

/**********************************************************************
* Function Name:    gnssConvEcef2NedVel
*
* Description:
*    convert ecef velocity to ENU velocity
*
* Input:
*     ecefVel: pointer to ECEF velocity
*     enuVel:  pointer to ENU velocity
*              0: east, 1: north, 2: up
*     lla:     pointer to lla position
*              0: latitude, 1: longitude, 2: altitude
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnssConvEcef2EnuVel(float* ecefVel, float* enuVel, double* lla)
{
  uint8_t i, j;
  float ceg[3][3];

  gnssGetEcef2EnuMatrix(ceg, lla);

  for (i = 0; i < 3; i++)
  {
    enuVel[i] = 0;

    for (j = 0; j < 3; j++)
    {
      enuVel[i] += ceg[i][j] * ecefVel[j];
    }
  }

  return;
}
/**********************************************************************
* Function Name:    gnssConvEnu2EcefVel
*
* Description:
*     convert enu velocity to ecef velocity
*
* Input:
*     enuVel:  pointer to ENU velocity
*     0: east, 1: north, 2: up
*     ecefVel: pointer to ECEF velocity
*     lla:     pointer to lla position
*              0: latitude, 1: longitude, 2: altitude
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnssConvEnu2EcefVel(float* enuVel, float* ecefVel, double* lla)
{
  uint8_t i, j;
  float ceg[3][3];
  float cge[3][3];

  gnssGetEcef2EnuMatrix(ceg, lla);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      cge[i][j] = ceg[j][i];
    }
  }

  for (i = 0; i < 3; i++)
  {
    ecefVel[i] = 0;

    for (j = 0; j < 3; j++)
    {
      ecefVel[i] += cge[i][j] * enuVel[j];
    }
  }

  return;
}
/***********************************************************************
* ��������: gnss_Kf_QualityControl
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/12/
***********************************************************************/
void gnss_Matrix_Mult(float a[3][3], float b[3][3], float c[3][3])
{
  uint8_t i, j, k;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      c[i][j] = 0.0;

      for (k = 0; k < 3; k++)
      {
        c[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}
/**********************************************************************
* Function Name:    gnssGetEnuPmatVar
*
* Description:
*    convert the covariance of P matrix frmo ECEF to ENU
*
* Input:
*     U:       upper triangular matrix of P, N*N
*     D:       diagonal matrix of UD decomposition, 1*N
*     Diag:    store the x,y,z,vx,vy,vz covariance in ENU coordinate
*     lla:     pointer to lla position
*              0: latitude, 1: longitude, 2: altitude
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnssGetEnuPmatVar(double* U, double* D, double* lla, double* diagTmp, float* Diag, float* enPosCov, float ecefPosP[6])
{
  uint32_t     i, j;
  float     ceg[3][3], cge[3][3];
  float     ecefPmat[N_STATE][N_STATE], ecefPosPMat[3][3] = { { 0.0 } }, enufPosPMat[3][3], ecefVelPMat[3][3] = { { 0.0 } }, enufVelPMat[3][3], matTmp[3][3];

  getUdMatP(U, D, ecefPmat, N_STATE);

  for (i = 0; i < N_STATE; i++)
  {
    diagTmp[i] = (double)ecefPmat[i][i];
  }

  //Convert ecef P mat to enu P mat
  gnssGetEcef2EnuMatrix(ceg, lla);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      cge[i][j] = ceg[j][i];
    }
  }

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      ecefPosPMat[i][j] = ecefPmat[i][j];
      ecefVelPMat[i][j] = ecefPmat[i + 3][j + 3];
    }
  }
  for (i = 0; i < 3; i++) ecefPosP[i] = ecefPosPMat[i][i];
  ecefPosP[3] = ecefPosPMat[0][1];
  ecefPosP[4] = ecefPosPMat[1][2];
  ecefPosP[5] = ecefPosPMat[0][2];

  gnss_Matrix_Mult(ceg, ecefPosPMat, matTmp);
  gnss_Matrix_Mult(matTmp, cge, enufPosPMat);

  gnss_Matrix_Mult(ceg, ecefVelPMat, matTmp);
  gnss_Matrix_Mult(matTmp, cge, enufVelPMat);

  for (i = 0; i < 3; i++)
  {
    Diag[i] = enufPosPMat[i][i];
    Diag[i + 3] = enufVelPMat[i][i];
  }
  *enPosCov = (float)enufPosPMat[0][1];
}


/**********************************************************************
* Function Name:    gnssClcSqrtAminusB
*
* Description:
*    calculate the sqrt(sum(A(i)^2 - B(i)^2))
*
* Input:
*     A:    vector A
*     B:    vector B
*     N:    number of components in A and B
* Return:
*     sqrt(sum(A(i)^2 - B(i)^2))
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnssClcSqrtAminusB_DBL(double* A, double* B, uint8_t N)
{
  uint8_t i;
  double rslt, tmp;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    tmp = A[i] - B[i];
    rslt += tmp * tmp;
  }

  rslt = sqrt(rslt);    // to be optimized by gnss math lib

  return rslt;
}

/**********************************************************************
* Function Name:    gnssClcSqrtAminusB_FLT
*
* Description:
*    calculate the sqrt(sum(A(i)^2 - B(i)^2))
*
* Input:
*     A:    vector A
*     B:    vector B
*     N:    number of components in A and B
* Return:
*     sqrt(sum(A(i)^2 - B(i)^2))
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
float gnssClcSqrtAminusB_FLT(float* A, float* B, uint8_t N)
{
  uint8_t i;
  float rslt, tmp;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    tmp = A[i] - B[i];
    rslt += tmp * tmp;
  }

  rslt = (float)sqrt(rslt);    // to be optimized by gnss math lib

  return rslt;
}
/**********************************************************************
* Function Name:    gnssClcAvg_DBL
*
* Description:
*    calculate the aveage of A
*
* Input:
*     A:    vector A
*     N:    number of components in A
* Return:
*     average(A(i)
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnssClcAvg_DBL(double* A, uint8_t N)
{
  uint8_t    i;
  double   rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i];
  }

  rslt = (double)(rslt / N);

  return rslt;
}
/**********************************************************************
* Function Name:    gnssClcAvg_FLT
*
* Description:
*    calculate the aveage of A
*
* Input:
*     A:    vector A
*     N:    number of components in A
* Return:
*     average(A(i)
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnssClcAvg_FLT(float* A, uint8_t N)
{
  uint8_t    i;
  float   rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i];
  }

  rslt = (float)(rslt / N);

  return rslt;
}

/**********************************************************************
* Function Name:    gnssClcSqrtSum_FLT
*
* Description:
*    calculate the sqrt(sum(A(i)^2)
*
* Input:
*     A:    vector A
*     N:    number of components in A
* Return:
*     sqrt(sum(A(i)^2)
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
float gnssClcSqrtSum_FLT(const float* A, uint8_t N)
{
  uint8_t i;
  float rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i] * A[i];
  }

  rslt = (float)sqrt(rslt);    // to be optimized by gnss math lib

  return (float)rslt;
}

/**********************************************************************
* Function Name:    gnssClcSqrtSum_DBL
*
* Description:
*    calculate the sqrt(sum(A(i)^2)
*
* Input:
*     A:    vector A
*     N:    number of components in A
* Return:
*     sqrt(sum(A(i)^2)
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnssClcSqrtSum_DBL(double* A, uint8_t N)
{
  uint8_t i;
  double rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i] * A[i];
  }

  rslt = (double)sqrt(rslt);

  return rslt;
}
/**********************************************************************
* Function Name:    gnssClcSquareSum_DBL
*
* Description:
*    calculate the sum(A(i)^2)
*
* Input:
*     A:    vector A
*     N:    number of components in A
* Return:
*     sum(A(i)^2
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnssClcSquareSum_DBL(double* A, uint8_t N)
{
  uint8_t i;
  double rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i] * A[i];
  }

  return rslt;
}
/**********************************************************************
* Function Name:    gnssClcVectorMulti
*
* Description:
*    calculate A*B
*
* Input:
*     A:    vector A
*     B:    vector B
*     N:    number of components in A
* Return:
*     A*B
*
* Dependency
*      None
*
* Author: juhong
**********************************************************************/
double  gnssClcVectorMulti(double* A, double* B, uint8_t N)
{
  uint8_t i;
  double rslt;

  rslt = 0;

  for (i = 0; i < N; i++)
  {
    rslt += A[i] * B[i];
  }

  return rslt;
}

/**********************************************************************
* Function Name:    gnssEarthRotateCorr
*
* Description:
*    correct satellite position due to earth rotation
*
* Input:
*     satPos:       satellite position
*     dt:           signal travel time
*
* Output:
*     satPosCorr:   satellite positon correction
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnssEarthRotateCorr(double* satPos, double* satPosCorr, double dt)
{
  double omegaTau = WGS84_OMEGDOTE * dt;
  double cosTau, sinTau;

  cosTau = 1;                 //approximate value
  sinTau = omegaTau;         //approximate value

  satPosCorr[0] = cosTau * satPos[0] + sinTau * satPos[1];
  satPosCorr[1] = -sinTau * satPos[0] + cosTau * satPos[1];
  satPosCorr[2] = satPos[2];
}

/**********************************************************************
* Function Name:    gnss_ls_SignFLT
*
* Description:
*       1. get the sign of float format
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int8_t gnss_SignFLT(float m_flt)
{
  if (m_flt < (float)0.0)
  {
    return -1;
  }
  else if (m_flt > (float)0.0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**********************************************************************
* Function Name:    gnss_QR_Factorize
*
* Description:
*
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gnss_QR_Factorize(float HMatrix[][15], uint8_t row, uint8_t colume)
{
  int32_t    i, j, k;
  float    s, r=0.0f, up, down;
  float* U;

  U = (float*)Sys_Malloc(sizeof(float) * row);

  if (U == NULL)
  {
    return 0;
  }

  memset(U, 0, sizeof(float) * row);

  if (U == NULL || row < colume)
  {
    //if (U)
    {
      Sys_Free(U);
    }

    return 0;
  }

  for (i = 0; i < colume; i++)
  {
    s = 0;

    for (k = i; k < row; k++)
    {
      s += HMatrix[k][i] * HMatrix[k][i];
    }

    if (HMatrix[i][i] >= 0)
    {
      s = (float)-sqrt(s);
    }
    else
    {
      s = (float)sqrt(s);
    }

    for (j = 0; j < row; j++)
    {
      U[i] = HMatrix[i][i] - s;

      if (j < i)
      {
        U[j] = 0;
      }
      else if (j > i)
      {
        U[j] = HMatrix[j][i];
      }
    }

    /*recompute the coefficient martrix*/
    /*the entries above the kth row will not change by multiplication of Q*/
    for (k = i + 1; k < colume; k++)
    {
      /*compute r for QV = V - R*U; now we have V and U. r = 2*U'*V/(U'*U)*/
      up = 0;
      down = 0;

      for (j = i; j < row; j++)
      {
        up += U[j] * HMatrix[j][k];
        down += U[j] * U[j];
      }

      if (down != 0.0)
      {
        r = 2 * up / down;
      }

      /*conpute the entries of column*/
      for (j = i; j < row; j++)
      {
        HMatrix[j][k] = HMatrix[j][k] - r * U[j];
      }
    }

    /*compute the ith column QV = V - U*/
    HMatrix[i][i] = s;

    /*if any one number in the diagonal is zero then this matrix is singular*/
    if (fabs(HMatrix[i][i]) < 1e-10)
    {
      Sys_Free(U);
      return 0;
    }

    for (j = i + 1; j < row; j++)
    {
      HMatrix[j][i] = 0;
    }
  }

  Sys_Free(U);
  return 1;
}

/**********************************************************************
* Function Name:    gnss_Inverse_UpMatrix
*
* Description:
*
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Inverse_UpMatrix(float HMatrix[][15], float invR[][15], uint8_t colume)
{
  int32_t    i, j, k;

  for (i = colume - 1; i >= 0; i--)
  {
    for (j = i; j < colume; j++)
    {
      if (j == i)
      {
        invR[i][j] = 1 / HMatrix[i][j];
      }
      else
      {
        invR[i][j] = 0;
        for (k = i + 1; k <= j; k++)
        {
          invR[i][j] += -(invR[k][j] * HMatrix[i][k]) / HMatrix[i][i];
        }
      }
    }
  }
}

/**********************************************************************
* Function Name:    gnss_UpMatrix_Square
*
* Description:
*
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_UpMatrix_Square(float invR[][15], float output[15][15], uint8_t colume)
{
  uint32_t    i, j, k;
  for (i = 0; i < colume; i++)
  {
    for (j = 0; j < colume; j++)
    {
      for (k = 0; k < colume; k++)
      {
        output[i][j] += invR[i][k] * invR[j][k];
      }
    }
  }
}


/**********************************************************************
* Function Name:    gnss_Sort_F
*
* Description:
*
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Sort_WithIndx(float* a, uint8_t* indx, uint32_t n)
{
  uint8_t        indxTmp;
  int32_t       i, j;
  int32_t       check;
  float       temp;
  if (n <= 1)
  {
    return;
  }
  for (i = n - 1; i > 0; i--)
  {
    check = 0;
    for (j = 0; j < i; j++)
    {
      if (a[j] > a[j + 1])
      {
        temp = a[j];
        a[j] = a[j + 1];
        a[j + 1] = temp;
        if (indx)
        {
          indxTmp = indx[j];
          indx[j] = indx[j + 1];
          indx[j + 1] = indxTmp;
        }
        check = 1;
      }
    }
    if (!check)
    {
      break;
    }
  }
}
/**********************************************************************
* Function Name:    gnss_Sort_WithIndx_1
*
* Description:
*
*
* Input:    float variable
*
* Output:   sign of variable;
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Sort_WithIndx_1(float* a, float* b, uint8_t* indx, uint32_t n)
{
  uint8_t        indxTmp;
  int32_t       i, j;
  int32_t       check;
  float       temp, temp1;
  if (n <= 1)
  {
    return;
  }
  for (i = n - 1; i > 0; i--)
  {
    check = 0;
    for (j = 0; j < i; j++)
    {
      if (a[j] > a[j + 1])
      {
        temp = a[j];
        a[j] = a[j + 1];
        a[j + 1] = temp;

        temp1 = b[j];
        b[j] = b[j + 1];
        b[j + 1] = temp1;

        indxTmp = indx[j];
        indx[j] = indx[j + 1];
        indx[j + 1] = indxTmp;
        check = 1;
      }
    }
    if (!check)
    {
      break;
    }
  }
}
/**********************************************************************
* Function Name:    gnss_Calc_distance
*
* Description:
*
*
* Input:    float variable
*
* Output:   float variable
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Calc_distance(float* a, float* b, uint32_t n)
{
  uint32_t  i, j;

  if (n <= 1)
  {
    return;
  }
  for (i = 0; i < n; i++)
  {
    b[i] = 0;
    for (j = 0; j < n; j++)
    {
      if (i == j) continue;
      b[i] += (float)fabsf(a[j] - a[i]);
    }
    b[i] /= (n - 1);
  }
}

float gnss_asinf(float f)
{
#ifdef EPSILON_ERROR_GUARD

  if ((f <= 1.0f) && (f >= -1.0f))
  {
    return (float)asin(f);
  }
  else if ((f > 1.0f) && (f <= (1.0f + (f_epsilon * EPSILON_ERROR_TOLERANCE))))
  {
    //SYS_LOGGING(OBJ"Float calculation issue");
    return (float)asin(1.0f);
  }
  else if ((f < -1.0f) && (f >= (-1.0f - (f_epsilon * EPSILON_ERROR_TOLERANCE))))
  {
    //gpsWarning("Float calculation issue");
    return (float)asin(-1.0f);
  }
  else
  {
    return (float)asin(f);
  }

#else
  return (float)asin(f);
#endif
}

/**********************************************************************
* Function Name:    gnss_math_fstd
*
* Description:
*
*
* Input:    float variable
*
* Output:   mean and std
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_math_fstd(float* data, uint32_t N, float* mean, float* std)
{
  uint32_t           i;
  float           sum = 0, tmp1;
  float           var = 0;

  if (N == 0)
  {
    (*mean) = 0;
    (*std) = 0;
    return;
  }

  for (i = 0; i < N; i++)
  {
    sum += data[i];
  }

  tmp1 = sum / N;

  for (i = 0; i < N; i++)
  {
    var += (data[i] - tmp1) * (data[i] - tmp1);
  }
  if (N > 1)
  {
    var /= (N - 1);
    (*mean) = tmp1;
    (*std) = (float)sqrt(var);
  }
  else
  {
    (*mean) = tmp1;
    (*std) = 0;
  }
}
/**********************************************************************
* Function Name:    gnss_math_dstd
*
* Description:
*
*
* Input:    double variable
*
* Output:   mean and std
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_math_dstd(double* data, uint32_t N, double* mean, double* std)
{
  uint32_t           i;
  double           sum = 0, tmp1;
  double           var = 0;

  if (N == 0)
  {
    (*mean) = 0;
    (*std) = 0;
    return;
  }

  for (i = 0; i < N; i++)
  {
    sum += data[i];
  }

  tmp1 = sum / N;

  for (i = 0; i < N; i++)
  {
    var += (data[i] - tmp1) * (data[i] - tmp1);
  }

  if (N > 1)
  {
    var /= (N - 1);
    (*mean) = tmp1;
    (*std) = (double)sqrt(var);
  }
  else
  {
    (*mean) = tmp1;
    (*std) = 0;
  }
}

static void gnss_swap(float* a, float* b)
{
  float c = *a;
  *a = *b; *b = c;
}

static void gnss_swap_dbl(double* a, double* b)
{
  double c = *a;
  *a = *b; *b = c;
}

static int32_t gnss_partition_dbl(double* DataVect, int32_t low, int32_t high)
{
  double data = DataVect[low];
  while (low < high)
  {
    while (low < high && DataVect[high] >= data)
      high--;
    gnss_swap_dbl(DataVect + low, DataVect + high);
    while (low < high && DataVect[low] <= data)
      low++;
    gnss_swap_dbl(DataVect + low, DataVect + high);
  }

  return low;
}

static int32_t gnss_partition(float* DataVect, int32_t low, int32_t high)
{
  uint32_t    loopCnt, loopCnt1, loopCnt2;
  uint32_t    maxLoopCnt = 100;
  float data = DataVect[low];

  loopCnt = 0;
  loopCnt1 = 0;
  loopCnt2 = 0;

  while (low < high)
  {
    while (low < high && DataVect[high] >= data)
    {
      high--;
      loopCnt1++;
      if (loopCnt1 > maxLoopCnt)
      {
        low = -1;
        return low;
      }
    }
    gnss_swap(DataVect + low, DataVect + high);
    while (low < high && DataVect[low] <= data)
    {
      low++;
      loopCnt2++;
      if (loopCnt2 > maxLoopCnt)
      {
        low = -1;
        return low;
      }
    }
    gnss_swap(DataVect + low, DataVect + high);

    loopCnt++;
    if (loopCnt > maxLoopCnt)
    {
      low = -1;
      return low;
    }
  }

  return low;
}

/**********************************************************************
* Function Name:    gnss_median_dbl
*
* Description:
*
*
* Input:
*
* Output:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gnss_median_dbl(double* data, uint32_t N, double* out)
{
  double* CheckCopy;
  /* from int32_t to uint32_t*/
  uint32_t       i;

  int32_t       left = 0, right = N - 1;
  int32_t       mid = (left + right) / 2;
  int32_t       pivotloc;
  double       median;
  double       MidP1;

  if (data == NULL || N <= 0)
  {
    *out = 0;
    return FALSE;
  }
  CheckCopy = (double*)Sys_Malloc(N * sizeof(double));
  if (CheckCopy == NULL)
  {
    *out = 0;
    return FALSE;
  }
  memcpy(CheckCopy, data, N * sizeof(double));


  while (1)
  {
    pivotloc = gnss_partition_dbl(CheckCopy, left, right);

    if (pivotloc == mid) break;
    if (pivotloc < mid) left = pivotloc + 1;
    if (pivotloc > mid) right = pivotloc - 1;
  }


  if (N & 0x01)
  {
    median = CheckCopy[mid];  // if Num is odd, return the order mid in the array
  }
  else							// if Num is even we must find the order mid + 1 in the array
  {								// the order mid +1 is the minimum value in the range [mid+1, Num-1] of current array
    median = CheckCopy[mid];
    MidP1 = CheckCopy[mid + 1];
    for (i = mid + 1; i < N; i++)
    {
      if (CheckCopy[i] < MidP1)  MidP1 = CheckCopy[i];
    }
    median = (median + MidP1) / 2;
  }
  *out = median;
  //if (CheckCopy) 
  {
    Sys_Free(CheckCopy);
  }
  return TRUE;
}
/**********************************************************************
* Function Name:    gnss_median
*
* Description:
*
*
* Input:
*
* Output:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gnss_median(float* data, uint32_t N, float* out)
{
  float* CheckCopy;
  uint32_t       loopCnt = 0, maxLoopCnt = 100;

  /* from int32_t to uint32_t*/
  uint32_t       i;

  int32_t       left = 0, right = N - 1;
  int32_t       mid = (left + right) / 2;
  int32_t       pivotloc;
  float       median;
  float       MidP1;

  if (data == NULL || N <= 0)
  {
    *out = 0;
    return FALSE;
  }
  CheckCopy = (float*)Sys_Malloc(N * sizeof(float));
  if (CheckCopy == NULL)
  {
    *out = 0;
    return FALSE;
  }
  memcpy(CheckCopy, data, N * sizeof(float));


  while (1)
  {
    pivotloc = gnss_partition(CheckCopy, left, right);
    if (pivotloc < 0)
    {
      Sys_Free(CheckCopy);
      return FALSE;
    }

    if (pivotloc == mid) break;
    if (pivotloc < mid) left = pivotloc + 1;
    if (pivotloc > mid) right = pivotloc - 1;

    loopCnt++;
    if (loopCnt > maxLoopCnt)
    {
      Sys_Free(CheckCopy);
      return FALSE;
    }
  }


  if (N & 0x01)
  {
    median = CheckCopy[mid];  // if Num is odd, return the order mid in the array
  }
  else							// if Num is even we must find the order mid + 1 in the array
  {								// the order mid +1 is the minimum value in the range [mid+1, Num-1] of current array
    median = CheckCopy[mid];
    MidP1 = CheckCopy[mid + 1];
    for (i = mid + 1; i < N; i++)
    {
      if (CheckCopy[i] < MidP1)  MidP1 = CheckCopy[i];
    }
    median = (median + MidP1) / 2;
  }
  *out = median;
  if (CheckCopy) Sys_Free(CheckCopy);
  return TRUE;
}

uint8_t gnss_MAD_DBL(double* data, double* dataAbs, uint8_t N, double* medianOut)
{
  uint8_t     i;
  uint8_t     flag;
  double    median;

  if (N < 1)
  {
    flag = FALSE;
    return flag;
  }

  *medianOut = 0;

  flag = gnss_median_dbl(data, N, &median);
  if (flag == FALSE) return flag;

  for (i = 0; i < N; i++)
  {
    dataAbs[i] = fabs(data[i] - median);
  }
  flag = gnss_median_dbl(dataAbs, N, &median);
  if (flag == TRUE) *medianOut = median;
  return flag;
}

uint8_t gnss_MAD(float* data, float* dataAbs, uint8_t N, float* medianOut)
{
  uint8_t     i;
  uint8_t     flag;
  float    median;

  if (N < 1)
  {
    flag = FALSE;
    return flag;
  }

  *medianOut = 0;

  flag = gnss_median(data, N, &median);
  if (flag == FALSE) return flag;

  for (i = 0; i < N; i++)
  {
    dataAbs[i] = (float)fabsf(data[i] - median);
  }
  flag = gnss_median(dataAbs, N, &median);
  if (flag == TRUE) *medianOut = median;
  return flag;
}

float gnss_lla2_heading(double* lla1, double* lla2)
{
  float      heading = 0.0;

  heading = (float)atan((lla2[1] - lla1[1]) * RAD2DEG * cos(lla2[0]) / ((lla2[0] - lla1[0]) * RAD2DEG));

  if (lla2[0] > lla1[0] && lla2[1] < lla1[1])
  {
    heading = 2 * (float)PI + heading;
  }

  if (lla2[0] < lla1[0])
  {
    heading = (float)PI + heading;
  }

  return heading;
}



/***********************************************************************
* ��������: gnss_kMean
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�3/17/
***********************************************************************/
uint8_t  gnss_kMean(uint32_t totalCnt, float* data, uint8_t* index_cluster, uint8_t flag)
{
  uint8_t             loop_cnt;
  uint32_t            i, j;
  uint32_t            cnt_k[3];
  float            mean_k[3], mean_last[3], dis[3];

  if (!flag)
  {
    mean_k[0] = data[0];
    mean_k[1] = data[totalCnt / 2];
    mean_k[2] = data[totalCnt - 1];
  }
  else
  {
    mean_k[0] = data[0];
    mean_k[1] = (float)((data[0] + data[totalCnt - 1]) / 2.0f);
    mean_k[2] = data[totalCnt - 1];
  }
  memset(mean_last, 0, 3 * sizeof(float));

  loop_cnt = 0;
  while (1)
  {
    //find the nearest mean center of each sample
    for (i = 0; i < totalCnt; i++)
    {
      index_cluster[i] = 0;
      for (j = 0; j < 3; j++)
      {
        dis[j] = (float)fabsf(data[i] - mean_k[j]);
        if (j > 0 && dis[j] < dis[j - 1])
        {
          index_cluster[i] = j;
        }
      }
    }

    for (j = 0; j < 3; j++)
    {
      mean_last[j] = mean_k[j];
      mean_k[j] = 0;
      cnt_k[j] = 0;
    }

    //calculate the new mean center
    for (i = 0; i < totalCnt; i++)
    {
      mean_k[index_cluster[i]] += data[i];
      cnt_k[index_cluster[i]]++;
    }

    for (j = 0; j < 3; j++)
    {
      if (cnt_k[j] > 0)
      {
        mean_k[j] /= cnt_k[j];
      }
      else
      {
        mean_k[j] = mean_last[j];
      }
    }

    loop_cnt++;

    //if the mean center doesn't change,then find the best mean center of each cluster
    if (fabsf(mean_k[0] - mean_last[0]) < 1e-5 && fabsf(mean_k[1] - mean_last[1]) < 1e-5 &&
      fabsf(mean_k[2] - mean_last[2]) < 1e-5)
    {
      break;
    }

    //if cluster fail, then return
    if (loop_cnt > 20)
    {
      return FALSE;
    }
  }
  return TRUE;
}
/***********************************************************************
* ��������: gnss_kMediods
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�3/17/
***********************************************************************/
uint8_t  gnss_kMediods(uint32_t totalCnt, float* data, uint8_t* index_cluster)
{
  uint8_t             loop_cnt;
  uint32_t            i, j;
  uint32_t            cnt_k[3];
  float            mean_k[3], mean_last[3], dis[3];
  float            data_cluster[3][MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { {0.0} };
  mean_k[0] = data[0];
  mean_k[1] = data[totalCnt / 2];
  mean_k[2] = data[totalCnt - 1];
  memset(mean_last, 0, 3 * sizeof(float));

  loop_cnt = 0;
  while (1)
  {
    //find the nearest mean center of each sample
    for (i = 0; i < totalCnt; i++)
    {
      index_cluster[i] = 0;
      for (j = 0; j < 3; j++)
      {
        dis[j] = (float)fabsf(data[i] - mean_k[j]);
        if (j > 0 && dis[j] < dis[j - 1])
        {
          index_cluster[i] = j;
        }
      }
    }

    for (j = 0; j < 3; j++)
    {
      mean_last[j] = mean_k[j];
      mean_k[j] = 0;
      cnt_k[j] = 0;
    }

    //calculate the new mean center
    for (i = 0; i < totalCnt; i++)
    {
      data_cluster[index_cluster[i]][cnt_k[index_cluster[i]]] = data[i];
      mean_k[index_cluster[i]] += data[i];
      cnt_k[index_cluster[i]]++;
    }

    for (j = 0; j < 3; j++)
    {
      if (cnt_k[j] > 0)
      {
        gnss_median(&(data_cluster[j][0]), cnt_k[j], &(mean_k[j]));
      }
      else
      {
        mean_k[j] = mean_last[j];
      }
    }

    loop_cnt++;

    //if the mean center doesn't change,then find the best mean center of each cluster
    if (fabsf(mean_k[0] - mean_last[0]) < 1e-5 && fabsf(mean_k[1] - mean_last[1]) < 1e-5 &&
      fabsf(mean_k[2] - mean_last[2]) < 1e-5)
    {
      break;
    }

    //if cluster fail, then return
    if (loop_cnt > 20)
    {
      return FALSE;
    }
  }
  return TRUE;
}

/***********************************************************************
* ��������: gnss_Cluster_Sort
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�12/16/
***********************************************************************/
void gnss_Cluster_Sort(float* dis_avg, uint32_t* cnt_cluster, uint32_t* cluster_indx, uint32_t* goodCnt, uint8_t cluster_N)
{
  uint8_t            i;
  uint32_t           cnt_tmp, cnt_good;
  float           dis_tmp;

  //find the best cluster
  cnt_tmp = cnt_cluster[0];
  cnt_good = goodCnt[0];
  dis_tmp = dis_avg[0];
  cluster_indx[0] = 0;
  for (i = 1; i < cluster_N; i++)
  {
    if (goodCnt[i] > cnt_good)
    {
      cnt_tmp = cnt_cluster[i];
      cnt_good = goodCnt[i];
      dis_tmp = dis_avg[i];
      cluster_indx[0] = i;
    }
    else if (goodCnt[i] == cnt_good)
    {
      if (cnt_cluster[i] > cnt_tmp)
      {
        cnt_tmp = cnt_cluster[i];
        cnt_good = goodCnt[i];
        dis_tmp = dis_avg[i];
        cluster_indx[0] = i;
      }
      else if (cnt_cluster[i] == cnt_tmp)
      {
        if (dis_avg[i] < dis_tmp)
        {
          cnt_tmp = cnt_cluster[i];
          cnt_good = goodCnt[i];
          dis_tmp = dis_avg[i];
          cluster_indx[0] = i;
        }
      }
    }
  }

  //find the worst cluster
  cnt_tmp = cnt_cluster[0];
  cnt_good = goodCnt[0];
  dis_tmp = dis_avg[0];
  cluster_indx[2] = 0;
  for (i = 1; i < cluster_N; i++)
  {
    if (goodCnt[i] < cnt_good)
    {
      cnt_tmp = cnt_cluster[i];
      cnt_good = goodCnt[i];
      dis_tmp = dis_avg[i];
      cluster_indx[2] = i;
    }
    else if (goodCnt[i] == cnt_good)
    {
      if (cnt_cluster[i] < cnt_tmp)
      {
        cnt_tmp = cnt_cluster[i];
        cnt_good = goodCnt[i];
        dis_tmp = dis_avg[i];
        cluster_indx[2] = i;
      }
      else if (cnt_cluster[i] == cnt_tmp)
      {
        if (dis_avg[i] > dis_tmp)
        {
          cnt_tmp = cnt_cluster[i];
          cnt_good = goodCnt[i];
          dis_tmp = dis_avg[i];
          cluster_indx[2] = i;
        }
      }
    }
  }
  if ((cluster_indx[0] + cluster_indx[2]) == 1)
  {
    cluster_indx[1] = 2;
  }
  else if ((cluster_indx[0] + cluster_indx[2]) == 2)
  {
    cluster_indx[1] = 1;
  }
  else
  {
    cluster_indx[1] = 0;
  }

}
void gnss_math_data_smooth(float dataInput, float* dataBack, uint8_t* dataBackCnt, uint8_t maxCnt, float* smoothData)
{
  uint8_t             i;
  uint8_t             cnt = 0;

  cnt = *dataBackCnt;

  if (cnt < maxCnt)
  {
    dataBack[cnt] = dataInput;
    cnt++;
    *dataBackCnt = cnt;
  }
  else
  {
    for (i = 0; i < maxCnt - 1; i++)
    {
      dataBack[i] = dataBack[i + 1];
    }

    dataBack[maxCnt - 1] = dataInput;
  }

  if (cnt == maxCnt)
  {
    *smoothData = (float)gnssClcAvg_FLT(dataBack, maxCnt);
  }
  else
  {
    *smoothData = dataInput;
  }
}

double gnss_math_min(double* data, int n)
{
  double min;
  int i;
  if (n < 1 || data == NULL) return 0.0;

  min = data[0];
  for (i = 1; i < n; i++)
  {
    if (min > data[i]) min = data[i];
  }
  return min;
}
/**********************************************************************
* Function Name:    Ins_Math_MatrixTrans
*
* Description:
*    b = a'
*    a is matrix with colume M, row N, b is matrix with colume N, row M
* Input:
*     a,N,M
*
* Return:
*     void
*
* Dependency
*      None
*
* Author: 
* Date: 11.28
**********************************************************************/
void gnss_Math_MatrixTrans(const double* a, uint8_t N, uint8_t M, double* b)
{
  uint8_t  i, j;

  if (N == 0 || M == 0 || a == NULL || b == NULL)
  {
    return;
  }

  for (i = 0; i < N; i++)
  {
    for (j = 0; j < M; j++)
    {
      b[j * 3 + i] = a[i * 3 + j];
    }
  }
}
/**********************************************************************
* Function Name:    Ins_Math_MatrixDot
*
* Description:
*    c = a * b
*    a is matrix with colume N1, row M1, b is matrix with colume N2, row M2
* Input:
*     a,b,Na,Ma,Nb,Mb
*
* Return:
*     void
*
* Dependency
*      None
*
* Author: 
* Date: 12.28
**********************************************************************/
void gnss_Math_MatrixDot(const double* a, uint8_t N1, uint8_t M1, const double* b, uint8_t N2, uint8_t M2, double* c)
{
  uint8_t  i, j, k;

  if (N1 == 0 || M1 == 0 || N2 == 0 || M2 == 0 || a == NULL || b == NULL || c == NULL)
  {
    return;
  }

  if (M1 != N2)
  {
    return;
  }

  for (i = 0; i < N1; i++)
  {
    for (j = 0; j < M2; j++)
    {
      c[i * M2 + j] = 0.0;
      for (k = 0; k < M1; k++)
      {
        c[i * M2 + j] += a[i * M1 + k] * b[k * M2 + j];
      }
    }
  }
}
void gnss_Math_MatrixDot_DF(const double* a, uint8_t N1, uint8_t M1, const float* b, uint8_t N2, uint8_t M2, float* c)
{
  uint8_t  i, j, k;

  if (N1 == 0 || M1 == 0 || N2 == 0 || M2 == 0 || a == NULL || b == NULL || c == NULL)
  {
    return;
  }

  if (M1 != N2)
  {
    return;
  }

  for (i = 0; i < N1; i++)
  {
    for (j = 0; j < M2; j++)
    {
      c[i * M2 + j] = 0.0;
      for (k = 0; k < M1; k++)
      {
        c[i * M2 + j] += (float)(a[i * M1 + k] * b[k * M2 + j]);
      }
    }
  }
}
/**
*  @Ins_Math_Euler2DCM: convert body frame to navigation frame
*  @author 
*  @date  01/02 16:01
*/
void gnss_Math_Euler2DCM(double* Euler, double* cbn)
{
  double           cosYaw, sinYaw;
  double           cosPitch, sinPitch;
  double           cosRoll, sinRoll;
  double           cnb0[9];
  // Yaw = Euler[0], pitch = Euler[1], roll = Euler[2]
  if (Euler == NULL || cbn == NULL)
  {
    return;
  }

  /*
  Body Frame: x --> right, y --> forward, z --> up
  Navigation Frame: x --> east, y --> North, z --> up
  */

  cosYaw = cos(Euler[0]);
  sinYaw = sin(Euler[0]);
  cosPitch = cos(Euler[1]);
  sinPitch = sin(Euler[1]);
  cosRoll = cos(Euler[2]);
  sinRoll = sin(Euler[2]);


  cnb0[0] = -sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
  cnb0[1] = sinYaw * cosRoll + cosYaw * sinPitch * sinRoll;
  cnb0[2] = -sinRoll * cosPitch;

  cnb0[3] = -sinYaw * cosPitch;
  cnb0[4] = cosYaw * cosPitch;
  cnb0[5] = sinPitch;

  cnb0[6] = sinRoll * cosYaw + cosRoll * sinPitch * sinYaw;
  cnb0[7] = sinRoll * sinYaw - cosRoll * sinPitch * cosYaw;
  cnb0[8] = cosPitch * cosRoll;


  cbn[0] = cnb0[0];
  cbn[1] = cnb0[3];
  cbn[2] = cnb0[6];

  cbn[3] = cnb0[1];
  cbn[4] = cnb0[4];
  cbn[5] = cnb0[7];

  cbn[6] = cnb0[2];
  cbn[7] = cnb0[5];
  cbn[8] = cnb0[8];

}