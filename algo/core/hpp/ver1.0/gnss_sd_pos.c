/************************************************************
* Copyrights(C)
* All rights Reserved
* 文件名称：gnss_sd_pos.c
* 文件描述：卫星位置以及钟差计算
* 版本号：1.0.0
* 作者：
* 日期：09/05
************************************************************/
#include "gnss_sd_pos.h"
#include "gnss_tm.h"
#include "gnss_math.h"
#include "gnss_hsm_lite_api.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SD

#define BDS_ID_IS_GEO(x)         (((x)>0&&x<=5)||((x)>=59&&(x)<=63))

extern GNSS_TIMESYSPARAM* p_gnssTimeSysParam;
extern Gnss_Cfg_t		g_pe_cfg;

//Inverse Matrix，20s Interval，Cubic interpolation
const double gnss_Sd_R_inv[4][4] =
{ {-2.08333333333333e-5,   6.24999999999999e-5,   -6.24999999999999e-5, 2.08333333333333e-5},
{               0.0025,              -0.00625,      0.005,            -0.00125},
{  -0.0916666666666666,                  0.15,     -0.075,  0.0166666666666667},
{                  1.0,                   0.0,        0.0,                 0.0} };

double gnss_Multiply(const double* R, double* B, uint8_t N)
{
  double X = 0.0;
  uint8_t i;

  for (i = 0; i < N; i++)
  {
    X += R[i] * B[i];
  }
  return X;
}
/**********************************************************************
* Function Name:    gps_sd_satpos_e
*
* Description:
*    gps_sd_satpos_e is function used to calculate GPS satellite position with EPH
*
* Input:
*    ed:     EPH data
*    t:       tot
*    svp:    calculated satellite position
*    sv_id: satellite ID
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gps_sd_satpos_e(GPS_EPH_INFO* pEph, double t, uint8_t sv_id, ECEF* svp, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  double      tk, a, b;
  double      Mk, Ek, phik, phik2, sin_phik2, cos_phik2;
  double      del_uk, del_rk, del_ik, rk;
  double      xkp, ykp, p0, p1;
  double      uk, sinuk, cosuk, ok, sinok, cosok, ik, sinik, cosik;
  float      rdot, udot, vxkp, vykp;
  double      sek;
  uint8_t       iterCnt;
  double      Ek_New, Ek_diff;

  if (pEph == NULL || svp == NULL || dClk == NULL)
  {
    return 0;
  }
  if ((tk = t - pEph->t_oe) > (double)SECS_IN_WEEK / 2.0)
  {
    tk -= (double)SECS_IN_WEEK;
  }
  else if (tk < (double)-SECS_IN_WEEK / 2.0)
  {
    tk += (double)SECS_IN_WEEK;
  }

  if (tk > 3600 + 120.0)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }
  /* time gap check */
  if (fabs(tk) > 7200 + 240.0)
  {
    return 0;
  }

  /* compute other parameters */
  pEph->IODE = pEph->subframe1.IODC & 0xFF;
  if (fabs(pEph->e) >= 1.0)
  {
    svp->have_spos = FRM_NULL;
    return 0;
  }
  pEph->r1me2 = sqrt((double)1.0 - (pEph->e * pEph->e));
  pEph->Axis = pEph->sqrt_A * pEph->sqrt_A;
  pEph->n = ((WGS84_SQRT_U) / ((double)pEph->Axis * (double)pEph->sqrt_A)) + (double)pEph->delta_n; /* in radians/sec */
  pEph->OMEGA_n = (double)pEph->OMEGA_0 - (WGS84_OMEGDOTE * (double)pEph->t_oe);
  pEph->ODOT_n = (double)pEph->OMEGADOT - WGS84_OMEGDOTE;
  pEph->Axe = (float)(pEph->Axis * pEph->e);
  pEph->CDTR = (float)(pEph->sqrt_A * pEph->e) * (float)(-4.442807633e-10);

  Mk = pEph->M_0 + (pEph->n * tk);

  /* do iteration to calculate Ek */
  Ek = Mk;
  iterCnt = 0;
  do
  {
    Ek_New = Mk + (pEph->e * sin(Ek));
    Ek_diff = fabs(Ek_New - Ek);
    Ek = Ek_New;
    iterCnt++;
  } while ((Ek_diff > 1.0E-10) && (iterCnt < 20));

  a = cos(Ek);
  b = (double)1.0 - (pEph->e * a);
  sek = sin(Ek);
  pEph->sinEk = sek;
  phik = atan2(pEph->r1me2 * sek, a - pEph->e) + pEph->omega;
  phik2 = phik + phik;
  sin_phik2 = sin(phik2);
  cos_phik2 = cos(phik2);
  del_uk = ((double)pEph->C_uc * cos_phik2) + ((double)pEph->C_us * sin_phik2);
  del_rk = ((double)pEph->C_rc * cos_phik2) + ((double)pEph->C_rs * sin_phik2);
  del_ik = ((double)pEph->C_ic * cos_phik2) + ((double)pEph->C_is * sin_phik2);
  uk = phik + del_uk;
  sinuk = sin(uk);
  cosuk = cos(uk);
  rk = (pEph->Axis * b) + del_rk;
  xkp = rk * cosuk;
  ykp = rk * sinuk;
  ik = pEph->i_0 + del_ik + (double)pEph->IDOT * tk;
  sinik = sin(ik);
  cosik = cos(ik);
  ok = pEph->OMEGA_n + pEph->ODOT_n * tk;
  sinok = sin(ok);
  cosok = cos(ok);
  svp->p[0] = xkp * cosok - ykp * (p0 = cosik * sinok);
  svp->p[1] = xkp * sinok + ykp * (p1 = cosik * cosok);
  svp->p[2] = ykp * sinik;
  svp->have_spos = FRM_EPH;
  /*  return if velocity computation not needed */
  /*  Compute velocity information based on Ephemeris */
  rdot = (float)((double)pEph->Axe * (a = pEph->n / b) * sek);
  udot = (float)(a * pEph->r1me2 / b);
  vxkp = (float)((double)rdot * cosuk - ykp * (double)udot);
  vykp = (float)((double)rdot * sinuk + xkp * (double)udot);
  svp->v[0] = (float)((double)vxkp * cosok - svp->p[1] * pEph->ODOT_n - (double)vykp * p0);
  svp->v[1] = (float)((double)vxkp * sinok + svp->p[0] * pEph->ODOT_n + (double)vykp * p1);
  svp->v[2] = (float)((double)vykp * sinik);
  svp->t = t;

  if (dClk)
  {
    if ((tk = t - pEph->subframe1.t_oc) > (double)SECS_IN_WEEK / 2.0)
    {
      tk -= (double)SECS_IN_WEEK;
    }
    else if (tk < (double)-SECS_IN_WEEK / 2.0)
    {
      tk += (double)SECS_IN_WEEK;
    }
    (*dClk) = pEph->subframe1.a_f0 + pEph->subframe1.a_f1 * tk + pEph->subframe1.a_f2 * tk * tk;
    *dClk -= 2.0 * sqrt(MU_GPS * pEph->Axis) * pEph->e * sek / SQR(LIGHT_SEC);
    if (freq_index == 0)
    {
      *dClk -= pEph->subframe1.T_GD;
    }
    else if (freq_index == 1)
    {
      *dClk -= pEph->subframe1.T_GD * (GPS_L1_CARRIER / GPS_L2_CARRIER) * (GPS_L1_CARRIER / GPS_L2_CARRIER);
    }

    if (dopp_corr)
    {
      (*dopp_corr) = pEph->subframe1.a_f1 + 2.0 * pEph->subframe1.a_f2 * tk;
    }
  }

  return 1;
}

uint8_t gps_sd_satpos_clk_dop(GPS_EPH_INFO* pEph, double t, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  double      tk;
  double      sek;
  uint8_t       iterCnt;
  double      Ek_New, Ek_diff;

  double      Mk, Ek;

  if (pEph == NULL || dClk == NULL)
  {
    return 0;
  }
  if ((tk = t - pEph->t_oe) > (double)SECS_IN_WEEK / 2.0)
  {
    tk -= (double)SECS_IN_WEEK;
  }
  else if (tk < (double)-SECS_IN_WEEK / 2.0)
  {
    tk += (double)SECS_IN_WEEK;
  }

  if (tk > 3600 + 120.0)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }
  /* time gap check */
  if (fabs(tk) > 7200 + 240.0)
  {
    return 0;
  }

  pEph->Axis = pEph->sqrt_A * pEph->sqrt_A;
  pEph->n = ((WGS84_SQRT_U) / ((double)pEph->Axis * (double)pEph->sqrt_A)) + (double)pEph->delta_n; /* in radians/sec */
  Mk = pEph->M_0 + (pEph->n * tk);

  /* do iteration to calculate Ek */
  Ek = Mk;
  iterCnt = 0;
  do
  {
    Ek_New = Mk + (pEph->e * sin(Ek));
    Ek_diff = fabs(Ek_New - Ek);
    Ek = Ek_New;
    iterCnt++;
  } while ((Ek_diff > 1.0E-10) && (iterCnt < 20));

  sek = sin(Ek);

  if (dClk)
  {
    if ((tk = t - pEph->subframe1.t_oc) > (double)SECS_IN_WEEK / 2.0)
    {
      tk -= (double)SECS_IN_WEEK;
    }
    else if (tk < (double)-SECS_IN_WEEK / 2.0)
    {
      tk += (double)SECS_IN_WEEK;
    }
    (*dClk) = pEph->subframe1.a_f0 + pEph->subframe1.a_f1 * tk + pEph->subframe1.a_f2 * tk * tk;
    *dClk -= 2.0 * sqrt(MU_GPS * pEph->Axis) * pEph->e * sek / SQR(LIGHT_SEC);
    if (freq_index == 0)
    {
      *dClk -= pEph->subframe1.T_GD;
    }
    else if (freq_index == 1)
    {
      *dClk -= pEph->subframe1.T_GD * (GPS_L1_CARRIER / GPS_L2_CARRIER) * (GPS_L1_CARRIER / GPS_L2_CARRIER);
    }

    if (dopp_corr)
    {
      (*dopp_corr) = pEph->subframe1.a_f1 + 2.0 * pEph->subframe1.a_f2 * tk;
    }
  }

  return 1;
}
/**********************************************************************
* Function Name:    dfunc
*
* Description:
*    dfunc
*
* Input:
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
static void dfunc(double* pVal0, double* pVal1)
{
  //double u = 398600.4418 * 1e9, ae = 6378136, j02 = 1082625.75 * 1e-9, w = 7.292115 * 1e-5;
  double r2 = pVal0[0] * pVal0[0] + pVal0[1] * pVal0[1] + pVal0[2] * pVal0[2];
  double r = sqrt(r2), r3 = r * r * r, r5 = r3 * r2;
  double c1 = MU_GLO / r3, c2 = 1.5 * J2_GLO * MU_GLO * RE_GLO * RE_GLO / r5, c3 = 5 * pVal0[2] * pVal0[2] / r2;
  double C1 = OMGE_GLO * OMGE_GLO - c1 - c2 * (1 - c3), C2 = -c1 - c2 * (3 - c3);
  pVal1[0] = pVal0[3];
  pVal1[1] = pVal0[4];
  pVal1[2] = pVal0[5];
  pVal1[3] = pVal0[6];
  pVal1[4] = pVal0[7];
  pVal1[5] = pVal0[8];
  pVal1[3] += C1 * pVal0[0] + 2 * OMGE_GLO * pVal0[4];
  pVal1[4] += C1 * pVal0[1] - 2 * OMGE_GLO * pVal0[3];
  pVal1[5] += C2 * pVal0[2];
}
/**********************************************************************
* Function Name:    rk4
*
* Description:
*    This function is RK4 used for GLN satellite calculation which is provided by ICD
*
* Input:
*
* Return:
*
*
* Dependency
*      None
*
* Author: wangys
**********************************************************************/
static void rk4(double h, double* pVal0, double* pVal1)
{
  uint8_t   i, j;
  double  C[4] = { 6, 3, 3, 6 };
  double  k[6][4];
  double  val[6];
  double* pVal = pVal0;
  double* pVali = &val[0];
  dfunc(pVal, pVali);

  for (i = 0; i < 6; i++)
  {
    k[i][0] = h * pVali[i];
    pVal1[i] = pVal0[i] + k[i][0] / 2;
  }

  pVal = pVal1;
  dfunc(pVal, pVali);

  for (i = 0; i < 6; i++)
  {
    k[i][1] = h * pVali[i];
    pVal1[i] = pVal0[i] + k[i][1] / 2;
  }

  dfunc(pVal, pVali);

  for (i = 0; i < 6; i++)
  {
    k[i][2] = h * pVali[i];
    pVal1[i] = pVal0[i] + k[i][2];
  }

  dfunc(pVal, pVali);

  for (i = 0; i < 6; i++)
  {
    k[i][3] = h * pVali[i];
  }

  for (i = 0; i < 6; i++)
  {
    pVal1[i] = pVal0[i];

    for (j = 0; j < 4; j++)
    {
      pVal1[i] += k[i][j] / C[j];
    }
  }
}

/**********************************************************************
* Function Name:    gln_sd_sv_pos
*
* Description:
*    gln_sd_sv_pos
*
* Input: t --> second of day
*
* Return:
*
*
* Dependency
*      None
*
* Author: wangys
**********************************************************************/
uint8_t gln_sd_sv_pos_e(uint8_t doExpireCheck, GLN_EPH_INFO* pEph, double t, ECEF* svpos, double* dClk, double* dopp_corr)
{
  uint8_t       i;
  double      h = 30;
  double      dt;
  double      ti;
  double      t1;
  double      valIn[9] = { 0.0 };
  double      valOut[9] = { 0.0 };
  double* pVal0 = &valIn[0], * pVal1 = &valOut[0];
  double* r;
  double* v;
  double* a;
  double      tIn = t;
  /* parameter check */
  if (pEph == NULL || svpos == NULL || dClk == NULL)
  {
    return 0;
  }

  /* Health check */
  if (pEph->Bn & 0x4)
  {
    return 0;
  }

  dt = t - pEph->tb;

  // over-roll check
  if (dt > (double)(SECS_IN_DAY / 2))
  {
    dt -= (double)SECS_IN_DAY;
    tIn -= (double)SECS_IN_DAY;
  }
  else if (dt < -(double)(SECS_IN_DAY / 2))
  {
    dt += (double)SECS_IN_DAY;
    tIn += (double)SECS_IN_DAY;
  }

  if (doExpireCheck && dt > (900 + 95.0))
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }

  /* time gap check */
  if (fabs(dt) > 1800)
  {
    return 0;
  }

  t1 = pEph->tb + (int32_t)(dt / h) * h;

  r = &pEph->r[0];
  v = &pEph->v[0];
  a = &pEph->a[0];

  for (i = 0; i < 3; i++)
  {
    valIn[i] = r[i];
    valIn[i + 3] = v[i];
    valIn[i + 6] = a[i];
  }

  if (t1 < pEph->tb)
  {
    h = -h;
  }

  if (h > 0)
  {
    for (ti = pEph->tb; ti <= t1 - h; ti += h)
    {
      rk4(h, pVal0, pVal1);

      for (i = 0; i < 6; i++)
      {
        pVal0[i] = pVal1[i];
      }
    }
  }
  else
  {
    for (ti = pEph->tb; ti >= t1 - h; ti += h)
    {
      rk4(h, pVal0, pVal1);

      for (i = 0; i < 6; i++)
      {
        pVal0[i] = pVal1[i];
      }
    }
  }

  // residual t propagation
  if (fabs(tIn - t1) > 0)
  {
    rk4(tIn - t1, pVal0, pVal1);
    for (i = 0; i < 6; i++)
    {
      pVal0[i] = pVal1[i];
    }
  }

  for (i = 0; i < 3; i++)
  {
    svpos->p[i] = pVal0[i];
    svpos->v[i] = (float)pVal0[i + 3];
    svpos->a[i] = (float)pVal0[i + 6];

  }
  svpos->t = t;
  svpos->have_spos = FRM_EPH;

  /* satellite clock bias and drift calculation */
  if (dClk)
  {
    *dClk = p_gnssTimeSysParam->tauGPS[GLN_MODE] + pEph->taun_tb - pEph->gaman_tb * dt;
    if (dopp_corr)
    {
      *dopp_corr = -pEph->gaman_tb;
    }
  }

  return 1;
}

uint8_t gln_sd_sv_clk_dop(uint8_t doExpireCheck, GLN_EPH_INFO* pEph, double t, double* dClk, double* dopp_corr)
{
  double      h = 30;
  double      dt;
  double      tIn = t;
  /* parameter check */
  if (pEph == NULL || dClk == NULL)
  {
    return 0;
  }

  /* Health check */
  if (pEph->Bn & 0x4)
  {
    return 0;
  }

  dt = t - pEph->tb;

  // over-roll check
  if (dt > (double)(SECS_IN_DAY / 2))
  {
    dt -= (double)SECS_IN_DAY;
    tIn -= (double)SECS_IN_DAY;
  }
  else if (dt < -(double)(SECS_IN_DAY / 2))
  {
    dt += (double)SECS_IN_DAY;
    tIn += (double)SECS_IN_DAY;
  }

  if (doExpireCheck && dt > (900 + 95.0))
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }

  /* time gap check */
  if (fabs(dt) > 1800)
  {
    return 0;
  }

  /* satellite clock bias and drift calculation */
  if (dClk)
  {
    *dClk = p_gnssTimeSysParam->tauGPS[GLN_MODE] + pEph->taun_tb - pEph->gaman_tb * dt;
    if (dopp_corr)
    {
      *dopp_corr = -pEph->gaman_tb;
    }
  }

  return 1;
}

void MtMul_MNxNL(double a[], double b[], double c[], int32_t m, int32_t n, int32_t l)
{
  int32_t  row, col, k;

  for (row = 0; row < m; row++)
  {
    for (col = 0; col < l; col++)
    {
      c[row * l + col] = 0.0;

      for (k = 0; k < n; k++)
      {
        c[row * l + col] += a[row * m + k] * b[k * l + col];
      }
    }
  }
}
/**********************************************************************
* Function Name:    bds_sd_sv_pos_e
*
* Description:
*    bds_sd_sv_pos_e
*
* Input: t --> second of week
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t bds_sd_sv_pos_e(BDS_EPH_INFO* pEph, double t, ECEF* svpos, uint8_t GEO, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  uint8_t      k = 0;
  double     dt = 0;
  double     M = 0;// the mean anomaly array
  double     NU;  // true anomaly array
  double     Radius = 0;
  double     lon = 0, lt = 0;
  double     inc = 0;
  double     xp = 0, yp = 0;
  double     Mdot, Edot, n, NUt, omigat, Ut, rt, iot, xpt, ypt;
  double     delta, E, guess, sinnu, cosnu, r0, omiga, U, phi;
  double     Pk[3] = { 0 }, Vk[3] = { 0 }, RR[9] = { 0 }, Rphi[9] = { 0 }, Rdp[9] = { 0 }, RRz[9] = { 0 };
  double     p[3], v[3];
  double     F;

  if (pEph == NULL || svpos == NULL || dClk == NULL)
  {
    return 0;
  }

  dt = t - pEph->toe;

  if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }
  else if (dt < -SECS_IN_WEEK / 2.0) // if into the last week
  {
    dt += (double)SECS_IN_WEEK;
  }

  if (dt > 3600 + 120.0)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }
  /* time gap check*/
  if (fabs(dt) > 3600 + 240.0)
  {
    return 0;
  }

  M = pEph->M0 + (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n) * dt;
  n = (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n);
  Mdot = (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n);
  guess = M;// initial value for E

  for (k = 0; k < 15; k++)
  {
    E = guess - (guess - pEph->ecc * sin(guess) - M) / (1 - pEph->ecc * cos(guess));
    delta = guess - E;
    guess = E;
    k++;
  }

  E = guess - (guess - pEph->ecc * sin(guess) - M) / (1 - pEph->ecc * cos(guess));
  Edot = Mdot / (1 - pEph->ecc * cos(E));
  if (fabs(pEph->ecc) >= 1.0)
  {
    svpos->have_spos = FRM_NULL;
    return 0;
  }
  sinnu = sqrt(1 - pEph->ecc * pEph->ecc) * sin(E);
  cosnu = cos(E) - pEph->ecc;
  NU = atan2(sinnu, cosnu);
  guess = pEph->w;
  k = 0;

  for (k = 0; k < 10; k++)
  {
    U = guess + NU;
    omiga = pEph->w + pEph->cuc * cos(2 * U) + pEph->cus * sin(2 * U);
    delta = omiga - guess;
    guess = omiga;
    k++;
  }

  omiga = pEph->w + pEph->cuc * cos(2 * U) + pEph->cus * sin(2 * U);
  U = omiga + NU;

  if (!GEO)
  {
    lon = pEph->OMEGA_0 + dt * (pEph->OMEGA_Dot - OMGE_BDS) - OMGE_BDS * pEph->toe;
    lt = pEph->OMEGA_Dot - OMGE_BDS;
  }
  else
  {
    lon = pEph->OMEGA_0 + pEph->OMEGA_Dot * dt - OMGE_BDS * pEph->toe;
    lt = pEph->OMEGA_Dot;
  }

  r0 = pEph->sqrta * pEph->sqrta * (1 - pEph->ecc * cos(E));
  Radius = r0 + pEph->crc * cos(2 * U) + pEph->crs * sin(2 * U);
  inc = pEph->i0 + pEph->idot * dt + pEph->cic * cos(2 * U)
    + pEph->cis * sin(2 * U);
  NUt = sin(E) * Edot * (1 + pEph->ecc * cos(NU)) / ((1 - pEph->ecc * cos(NU)) * sin(NU));
  omigat = (2.0 * (pEph->cus * cos(2 * U) - pEph->cuc * sin(2 * U))) * NUt / (1 - 2.0 * (pEph->cus * cos(2 * U) - pEph->cuc * sin(2 * U)));
  //
  Ut = (2.0 * (pEph->cus * cos(2 * U) - pEph->cuc * sin(2 * U))) * NUt + NUt;
  rt = pEph->sqrta * pEph->sqrta * pEph->ecc * sin(E) * Edot + 2.0 * (pEph->crs * cos(2 * U) - pEph->crc * sin(2 * U)) * NUt;
  iot = pEph->idot + 2.0 * (pEph->cis * cos(2 * U) - pEph->cic * sin(2 * U)) * NUt;
  xp = Radius * cos(U);
  yp = Radius * sin(U);
  p[0] = xp * cos(lon) - (yp * cos(inc) * sin(lon));
  p[1] = xp * sin(lon) + yp * cos(inc) * cos(lon);
  p[2] = yp * sin(inc);
  xpt = rt * cos(U) - yp * Ut;
  ypt = rt * sin(U) + xp * Ut;
  v[0] = (float)((xpt - yp * cos(inc) * lt) * cos(lon) - (xp * lt + ypt * cos(inc) - yp * sin(inc) * iot) * sin(lon));
  v[1] = (float)((xpt - yp * cos(inc) * lt) * sin(lon) + (xp * lt + ypt * cos(inc) - yp * sin(inc) * iot) * cos(lon));
  v[2] = (float)(ypt * sin(inc) + yp * cos(inc) * iot);

  if (GEO)
  {
    RR[0] = 1;
    RR[4] = cos(GEOROTATE);
    RR[5] = sin(GEOROTATE);
    RR[7] = -sin(GEOROTATE);
    RR[8] = cos(GEOROTATE);
    phi = OMGE_BDS * dt;
    Rphi[0] = cos(phi);
    Rphi[1] = sin(phi);
    Rphi[3] = -sin(phi);
    Rphi[4] = cos(phi);
    Rphi[8] = 1;
    Rdp[0] = -sin(phi) * OMGE_BDS;
    Rdp[1] = cos(phi) * OMGE_BDS;
    Rdp[3] = -cos(phi) * OMGE_BDS;
    Rdp[4] = -sin(phi) * OMGE_BDS;
    MtMul_MNxNL(Rdp, RR, RRz, 3, 3, 3);
    MtMul_MNxNL(RRz, p, Pk, 3, 3, 1);
    MtMul_MNxNL(Rphi, RR, RRz, 3, 3, 3);
    MtMul_MNxNL(RRz, v, Vk, 3, 3, 1);
    v[0] = Vk[0] + Pk[0];
    v[1] = Vk[1] + Pk[1];
    v[2] = Vk[2] + Pk[2];
    MtMul_MNxNL(RRz, p, Pk, 3, 3, 1);
    p[0] = Pk[0];
    p[1] = Pk[1];
    p[2] = Pk[2];
  }

  for (k = 0; k < 3; k++)
  {
    svpos->p[k] = p[k];
    svpos->v[k] = (float)v[k];
  }
  svpos->have_spos = FRM_EPH;

  dt = t - pEph->toc;
  if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }
  else if (dt < -SECS_IN_WEEK / 2.0)
  {
    dt += (double)SECS_IN_WEEK;
  }
  if (fabs(dt) > 3600 + 240.0)
  {
    pEph->eph_status = EPH_STATUS_INVALID;
    return 0;
  }

  pEph->sinEk = sin(E);
  if (dClk)
  {
    *dClk = pEph->af0 + pEph->af1 * (dt)+pEph->af2 * dt * dt;
    F = -2.0 * sqrt(MU_BDS) / (LIGHT_SEC * LIGHT_SEC);
    (*dClk) += F * pEph->ecc * pEph->sqrta * pEph->sinEk;
    if (freq_index == 0)
    {
      (*dClk) -= pEph->TGD1/* * 1e-9*/;
    }
#if 0 // now B3I saved in freq_index=1, so do not need to correct tgd; B2I need to correct
    else if (freq_index == 1)
    {
      (*dClk) -= pEph->TGD2 /** 1e-9*/;
    }
#endif
    if (fabs(*dClk) >= 1.0)
    {
      pEph->eph_status = EPH_STATUS_INVALID;
      return 0;
    }

    if (dopp_corr)
    {
      *dopp_corr = pEph->af1 + 2.0 * pEph->af2 * dt;
    }
  }

  return 1;
}

uint8_t bds_sd_sv_clk_dop(BDS_EPH_INFO* pEph, double t, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  uint8_t      k = 0;
  double     dt = 0;
  double     Mdot, n;
  double     M = 0;// the mean anomaly array
  double     delta, E, guess;
  double     F;

  if (pEph == NULL || dClk == NULL)
  {
    return 0;
  }

  dt = t - pEph->toe;

  if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }
  else if (dt < -SECS_IN_WEEK / 2.0) // if into the last week
  {
    dt += (double)SECS_IN_WEEK;
  }

  if (dt > 3600 + 120.0)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }
  /* time gap check*/
  if (fabs(dt) > 3600 + 240.0)
  {
    return 0;
  }

  M = pEph->M0 + (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n) * dt;
  n = (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n);
  Mdot = (sqrt(MU_BDS) * pow(pEph->sqrta, -3) + pEph->delta_n);
  guess = M;// initial value for E

  for (k = 0; k < 15; k++)
  {
    E = guess - (guess - pEph->ecc * sin(guess) - M) / (1 - pEph->ecc * cos(guess));
    delta = guess - E;
    guess = E;
    k++;
  }

  E = guess - (guess - pEph->ecc * sin(guess) - M) / (1 - pEph->ecc * cos(guess));

  dt = t - pEph->toc;
  if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }
  else if (dt < -SECS_IN_WEEK / 2.0)
  {
    dt += (double)SECS_IN_WEEK;
  }
  if (fabs(dt) > 3600 + 240.0)
  {
    pEph->eph_status = EPH_STATUS_INVALID;
    return 0;
  }

  pEph->sinEk = sin(E);
  if (dClk)
  {
    *dClk = pEph->af0 + pEph->af1 * (dt)+pEph->af2 * dt * dt;
    F = -2.0 * sqrt(MU_BDS) / (LIGHT_SEC * LIGHT_SEC);
    (*dClk) += F * pEph->ecc * pEph->sqrta * pEph->sinEk;
    if (freq_index == 0)
    {
      (*dClk) -= pEph->TGD1 /** 1e-9*/;
    }
#if 0 // now B3I saved in freq_index=1, so do not need to correct tgd; B2I need to correct
    else if (freq_index == 1)
    {
      (*dClk) -= pEph->TGD2 /** 1e-9*/;
    }
#endif
    if (fabs(*dClk) >= 1.0)
    {
      pEph->eph_status = EPH_STATUS_INVALID;
      return 0;
    }

    if (dopp_corr)
    {
      *dopp_corr = pEph->af1 + 2.0 * pEph->af2 * dt;
    }
  }

  return 1;
}

/**********************************************************************
* Function Name:    gal_sd_sv_pos_e
*
* Description:
*    gal_sd_sv_pos_e
*
* Input: t --> second of week
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gal_sd_sv_pos_e(GAL_EPH_INFO* pEph, double t, ECEF* svp, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  double      tk, a, b;
  double      Mk, Ek, phik, phik2, sin_phik2, cos_phik2;
  double      del_uk, del_rk, del_ik, rk;
  double      xkp, ykp, p0, p1;
  double      uk, sinuk, cosuk, ok, sinok, cosok, ik, sinik, cosik;
  float      rdot, udot, vxkp, vykp;
  double      sek;
  uint8_t       iterCnt;
  double      Ek_New, Ek_diff;

  if (pEph == NULL || svp == NULL || dClk == NULL)
  {
    return 0;
  }
  if ((tk = t - pEph->toe) > (double)SECS_IN_WEEK / 2.0)
  {
    tk -= (double)SECS_IN_WEEK;
  }
  else if (tk < (double)-SECS_IN_WEEK / 2.0)
  {
    tk += (double)SECS_IN_WEEK;
  }

  if (tk > 3600)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }

  /* time gap check */
  if (fabs(tk) > 7200)
  {
    return 0;
  }

  /* compute other parameters */
  //pEph->IODE = pEph->subframe1.IODC & 0xFF;
  if (fabs(pEph->ecc) >= 1.0)
  {
    svp->have_spos = FRM_NULL;
    return 0;
  }
  pEph->r1me2 = sqrt((double)1.0 - (pEph->ecc * pEph->ecc));
  pEph->Axis = pEph->sqrta * pEph->sqrta;
  pEph->n = ((GTRF_SQRT_U) / ((double)pEph->Axis * (double)pEph->sqrta)) + (double)pEph->delta_n; /* in radians/sec */
  pEph->OMEGA_n = (double)pEph->OMEGA_0 - (WGS84_OMEGDOTE * (double)pEph->toe);
  pEph->ODOT_n = (double)pEph->OMEGA_Dot - WGS84_OMEGDOTE;
  pEph->Axe = (float)(pEph->Axis * pEph->ecc);
  pEph->CDTR = (float)(pEph->sqrta * pEph->ecc) * (float)(-4.442807633e-10);

  Mk = pEph->M0 + (pEph->n * tk);

  /* do iteration to calculate Ek */
  Ek = Mk;
  iterCnt = 0;
  do
  {
    Ek_New = Mk + (pEph->ecc * sin(Ek));
    Ek_diff = fabs(Ek_New - Ek);
    Ek = Ek_New;
    iterCnt++;
  } while ((Ek_diff > 1.0E-10) && (iterCnt < 20));

  a = cos(Ek);
  b = (double)1.0 - (pEph->ecc * a);
  sek = sin(Ek);
  pEph->sinEk = sek;
  phik = atan2(pEph->r1me2 * sek, a - pEph->ecc) + pEph->w;
  phik2 = phik + phik;
  sin_phik2 = sin(phik2);
  cos_phik2 = cos(phik2);
  del_uk = ((double)pEph->cuc * cos_phik2) + ((double)pEph->cus * sin_phik2);
  del_rk = ((double)pEph->crc * cos_phik2) + ((double)pEph->crs * sin_phik2);
  del_ik = ((double)pEph->cic * cos_phik2) + ((double)pEph->cis * sin_phik2);
  uk = phik + del_uk;
  sinuk = sin(uk);
  cosuk = cos(uk);
  rk = (pEph->Axis * b) + del_rk;
  xkp = rk * cosuk;
  ykp = rk * sinuk;
  ik = pEph->i0 + del_ik + (double)pEph->idot * tk;
  sinik = sin(ik);
  cosik = cos(ik);
  ok = pEph->OMEGA_n + pEph->ODOT_n * tk;
  sinok = sin(ok);
  cosok = cos(ok);
  svp->p[0] = xkp * cosok - ykp * (p0 = cosik * sinok);
  svp->p[1] = xkp * sinok + ykp * (p1 = cosik * cosok);
  svp->p[2] = ykp * sinik;
  svp->have_spos = FRM_EPH;
  /*  return if velocity computation not needed */
  /*  Compute velocity information based on Ephemeris */
  rdot = (float)((double)pEph->Axe * (a = pEph->n / b) * sek);
  udot = (float)(a * pEph->r1me2 / b);
  vxkp = (float)((double)rdot * cosuk - ykp * (double)udot);
  vykp = (float)((double)rdot * sinuk + xkp * (double)udot);
  svp->v[0] = (float)((double)vxkp * cosok - svp->p[1] * pEph->ODOT_n - (double)vykp * p0);
  svp->v[1] = (float)((double)vxkp * sinok + svp->p[0] * pEph->ODOT_n + (double)vykp * p1);
  svp->v[2] = (float)((double)vykp * sinik);
  svp->t = t;

  if (dClk)
  {
    if ((tk = t - pEph->toc) > (double)SECS_IN_WEEK / 2.0)
    {
      tk -= (double)SECS_IN_WEEK;
    }
    else if (tk < (double)-SECS_IN_WEEK / 2.0)
    {
      tk += (double)SECS_IN_WEEK;
    }
    (*dClk) = pEph->af0 + pEph->af1 * tk + pEph->af2 * tk * tk;
    *dClk -= 2.0 * sqrt(MU_GAL * pEph->Axis) * pEph->ecc * sek / SQR(LIGHT_SEC);

    if (!DBL_IS_EQUAL(pEph->bgd_e5a, 0.0) && freq_index == 2 && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_QCOM_855)//FNAV
    {
      *dClk -= pEph->bgd_e5a * (GAL_E1_CARRIER / GAL_E5a_CARRIER) * (GAL_E1_CARRIER / GAL_E5a_CARRIER);  //for NAV/F in signal E5a
    }
    else
    {
      if (freq_index == 0)
      {
        *dClk -= pEph->bgd_e5b;  //for NAV/I in signal E5b
      }
      if (freq_index == 1)
      {
        *dClk -= pEph->bgd_e5b * (GAL_E1_CARRIER / GAL_E5b_CARRIER) * (GAL_E1_CARRIER / GAL_E5b_CARRIER);  //for NAV/I in signal E5b
      }
    }



    if (dopp_corr)
    {
      (*dopp_corr) = pEph->af1 + 2.0 * pEph->af2 * tk;
    }
  }

  return 1;
}

uint8_t gal_sd_sv_clk_dop(GAL_EPH_INFO* pEph, double t, double* dClk, double* dopp_corr, uint32_t freq_index)
{
  double      tk;
  double      Mk, Ek;
  double      sek;
  uint8_t       iterCnt;
  double      Ek_New, Ek_diff;

  if (pEph == NULL || dClk == NULL)
  {
    return 0;
  }
  if ((tk = t - pEph->toe) > (double)SECS_IN_WEEK / 2.0)
  {
    tk -= (double)SECS_IN_WEEK;
  }
  else if (tk < (double)-SECS_IN_WEEK / 2.0)
  {
    tk += (double)SECS_IN_WEEK;
  }

  if (tk > 3600)
  {
    pEph->eph_status = EPH_STATUS_EXPIRE;
  }

  /* time gap check */
  if (fabs(tk) > 7200)
  {
    return 0;
  }

  Mk = pEph->M0 + (pEph->n * tk);

  /* do iteration to calculate Ek */
  Ek = Mk;
  iterCnt = 0;
  do
  {
    Ek_New = Mk + (pEph->ecc * sin(Ek));
    Ek_diff = fabs(Ek_New - Ek);
    Ek = Ek_New;
    iterCnt++;
  } while ((Ek_diff > 1.0E-10) && (iterCnt < 20));

  sek = sin(Ek);
  pEph->Axis = pEph->sqrta * pEph->sqrta;

  if (dClk)
  {
    if ((tk = t - pEph->toc) > (double)SECS_IN_WEEK / 2.0)
    {
      tk -= (double)SECS_IN_WEEK;
    }
    else if (tk < (double)-SECS_IN_WEEK / 2.0)
    {
      tk += (double)SECS_IN_WEEK;
    }
    (*dClk) = pEph->af0 + pEph->af1 * tk + pEph->af2 * tk * tk;
    *dClk -= 2.0 * sqrt(MU_GAL * pEph->Axis) * pEph->ecc * sek / SQR(LIGHT_SEC);
    if (!DBL_IS_EQUAL(pEph->bgd_e5a, 0.0) && freq_index == 2 && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_QCOM_855)//FNAV
    {
      *dClk -= pEph->bgd_e5a * (GAL_E1_CARRIER / GAL_E5a_CARRIER) * (GAL_E1_CARRIER / GAL_E5a_CARRIER);  //for NAV/F in signal E5a
    }
    else
    {
      if (freq_index == 0)
      {
        *dClk -= pEph->bgd_e5b;  //for NAV/I in signal E5b
      }
      if (freq_index == 1)
      {
        *dClk -= pEph->bgd_e5b * (GAL_E1_CARRIER / GAL_E5b_CARRIER) * (GAL_E1_CARRIER / GAL_E5b_CARRIER);  //for NAV/I in signal E5b
      }
    }

    if (dopp_corr)
    {
      (*dopp_corr) = pEph->af1 + 2.0 * pEph->af2 * tk;
    }
  }

  return 1;
}
double gnss_Sd_Pos_e1(uint8_t gnssMode, uint8_t sv_id, double t, ECEF* svp, double* dopp_corr, uint32_t freq_index)
{
  double               dClk = 0.0;
  void* pEph;
  GPS_EPH_INFO* pGpsEph = NULL;
  GLN_EPH_INFO* pGlnEph = NULL;
  BDS_EPH_INFO* pBdsEph = NULL;
  GAL_EPH_INFO* pGalEph = NULL;

  svp->have_spos = FRM_NULL;
  /* Get the EPH */
  pEph = gnss_Sd_Nm_GetEph(gnssMode, sv_id);
  if (pEph == NULL)
  {
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "There is no EPH for gnss(%02d) prn(%02d)", gnssMode, sv_id);
    return dClk;
  }

  /* Calculate satellite position and velocity */
  if (gnssMode == GPS_MODE)
  {
    pGpsEph = (GPS_EPH_INFO*)pEph;
    if (pGpsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;
    gps_sd_satpos_e(pGpsEph, t, sv_id, svp, &dClk, dopp_corr, freq_index);
  }
  else if (gnssMode == GLN_MODE)
  {
    pGlnEph = (GLN_EPH_INFO*)pEph;
    if (pGlnEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;
    gln_sd_sv_pos_e(1, pGlnEph, t, svp, &dClk, dopp_corr);
  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsEph = (BDS_EPH_INFO*)pEph;
    if (pBdsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;
    bds_sd_sv_pos_e(pBdsEph, t, svp, BDS_ID_IS_GEO(sv_id), &dClk, dopp_corr, freq_index);
  }
  else if (gnssMode == GAL_MODE)
  {
    pGalEph = (GAL_EPH_INFO*)pEph;
    if (pGalEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;
    gal_sd_sv_pos_e(pGalEph, t, svp, &dClk, dopp_corr, freq_index);
  }
  else
  {
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "Wrong gnssMode in %s", __FUNCTION__);
  }
RET_PLACE:
  return dClk;
}
void gnss_Construct_R(double dt, double* R)
{
  R[0] = dt * dt * dt;
  R[1] = dt * dt;
  R[2] = dt;
  R[3] = 1;
}

void gnss_Copy_Y_XYZ(double* Y_XYZ_pos, double* Y_XYZ_vel, ECEF* svp, uint8_t i)
{
  uint8_t j;
  for (j = 0; j < 3; j++) {//X Y Z
    Y_XYZ_pos[i + 4 * j] = svp->p[j];
    Y_XYZ_vel[i + 4 * j] = svp->v[j];
  }
}

//计算系数矩阵
void gnss_Calc_B_XYZ(double Y_XYZ_pos[], double Y_XYZ_vel[], sat_data_t* sp)
{
  uint8_t i, j;
  for (j = 0; j < 3; j++) {
    for (i = 0; i < 4; i++) {
      sp->B_XYZ_pos[i + 4 * j] = gnss_Multiply(gnss_Sd_R_inv[i], &Y_XYZ_pos[4 * j], 4);
      sp->B_XYZ_vel[i + 4 * j] = gnss_Multiply(gnss_Sd_R_inv[i], &Y_XYZ_vel[4 * j], 4);
    }
  }
}

/***********************************************************************
* 函数介绍: gnss_Sd_Pos_e
* 输入参数：gnssMode: 0 --> GPS, 1--> GLN, 2--> BDS
*           sv_id: PRN
*           t: tot, for GPS/BDS, it's second of week, for GLN, it's second of day
* 输出参数：svp: satellite position and velocity
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
double gnss_Sd_Pos_e(uint8_t gnssMode, uint8_t sv_id, double t, ECEF* svp, double* dopp_corr, uint32_t freq_index)
{
  uint8_t                i;
  uint8_t                j;
  double               dt = 0.0;
  double               R[4];
  double				  Y_XYZ_pos[12];
  double				  Y_XYZ_vel[12];
  double               dClk = 0.0;
  void* pEph;
  GPS_EPH_INFO* pGpsEph = NULL;
  GLN_EPH_INFO* pGlnEph = NULL;
  BDS_EPH_INFO* pBdsEph = NULL;
  GAL_EPH_INFO* pGalEph = NULL;
  sat_data_t* sp = NULL;

  svp->have_spos = FRM_NULL;
  /* Get the EPH */
  pEph = gnss_Sd_Nm_GetEph(gnssMode, sv_id);
  if (pEph == NULL)
  {
    GLOGI("There is no EPH for gnss(%02d) prn(%02d)", gnssMode, sv_id);
    return dClk;
  }

  sp = gnss_sd_get_sv_data(gnssMode, sv_id, freq_index);
  if (sp == NULL)
  {
    GLOGI("There is no EPH for gnss(%02d) prn(%02d)", gnssMode, sv_id);
    return dClk;
  }

  /* Calculate satellite position and velocity */
  if (gnssMode == GPS_MODE)
  {
    pGpsEph = (GPS_EPH_INFO*)pEph;
    if (pGpsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    dt = t - sp->fit_tm_start;
    if (dt < -SECS_IN_WEEK / 2.0)
    {
      dt += (double)SECS_IN_WEEK;
    }

    if (fabs(dt) >= 60 || sp->toe != pGpsEph->t_oe)
    {	//check if need re-calculate fitting coefficient
      sp->fit_flag = 1;
      sp->fit_tm_start = t;//start time of fitting
      sp->toe = pGpsEph->t_oe;//eph toe of fitting
      dt = 0.0;

      // select four epoch,interval = 20s
      for (i = 0; i < 4; i++)
      {
        if (!gps_sd_satpos_e(pGpsEph, t + i * 20.0, sv_id, svp, &dClk, dopp_corr, freq_index))
        {
          sp->fit_flag = 0;//fitting fail
          if (i != 0) //check if need re-calculate
          {
            gps_sd_satpos_e(pGpsEph, t, sv_id, svp, &dClk, dopp_corr, freq_index);
          }
          return dClk;
        }
        gnss_Copy_Y_XYZ(Y_XYZ_pos, Y_XYZ_vel, svp, i);//save sat pos/vel
      }
      gnss_Calc_B_XYZ(Y_XYZ_pos, Y_XYZ_vel, sp);//calculate coefficient matrix
    }
    else if (!sp->fit_flag)//use eph if fitting fail
    {
      gps_sd_satpos_e(pGpsEph, t, sv_id, svp, &dClk, dopp_corr, freq_index);
      return dClk;
    }
    gps_sd_satpos_clk_dop(pGpsEph, t, &dClk, dopp_corr, freq_index);//calculate sat clock bias/drift using EPH

  }
  else if (gnssMode == GLN_MODE)
  {
    pGlnEph = (GLN_EPH_INFO*)pEph;
    if (pGlnEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    dt = t - sp->fit_tm_start;
    if (dt < -SECS_IN_DAY / 2.0)
    {
      dt += (double)SECS_IN_DAY;
    }

    if (fabs(dt) >= 60 || sp->toe != pGlnEph->tb)
    {
      sp->fit_flag = 1;
      sp->fit_tm_start = t;
      sp->toe = (float)pGlnEph->tb;
      dt = 0.0;

      for (i = 0; i < 4; i++)//four epoch
      {
        if (!gln_sd_sv_pos_e(1, pGlnEph, t + i * 20.0, svp, &dClk, dopp_corr))//interval is 20s
        {
          sp->fit_flag = 0;
          if (i != 0) //check if need re-calculate
          {
            gln_sd_sv_pos_e(1, pGlnEph, t, svp, &dClk, dopp_corr);
          }
          return dClk;
        }
        gnss_Copy_Y_XYZ(Y_XYZ_pos, Y_XYZ_vel, svp, i);
      }
      gnss_Calc_B_XYZ(Y_XYZ_pos, Y_XYZ_vel, sp);// calculate coefficient matrix
    }
    else if (!sp->fit_flag)
    {
      gln_sd_sv_pos_e(1, pGlnEph, t, svp, &dClk, dopp_corr);
      return dClk;
    }
    gln_sd_sv_clk_dop(1, pGlnEph, t, &dClk, dopp_corr);
  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsEph = (BDS_EPH_INFO*)pEph;
    if (pBdsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    dt = t - sp->fit_tm_start;
    if (dt < -SECS_IN_WEEK / 2.0)
    {
      dt += (double)SECS_IN_WEEK;
    }

    if (fabs(dt) >= 60 || sp->toe != pBdsEph->toe) {
      sp->fit_flag = 1;
      sp->fit_tm_start = t;
      sp->toe = (float)pBdsEph->toe;
      dt = 0.0;

      for (i = 0; i < 4; i++)
      {
        if (!bds_sd_sv_pos_e(pBdsEph, t + i * 20.0, svp, BDS_ID_IS_GEO(sv_id), &dClk, dopp_corr, freq_index))
        {
          sp->fit_flag = 0;

          //check if need re-calculate
          if (i != 0)
          {
            bds_sd_sv_pos_e(pBdsEph, t, svp, BDS_ID_IS_GEO(sv_id), &dClk, dopp_corr, freq_index);
          }
          return dClk;
        }
        gnss_Copy_Y_XYZ(Y_XYZ_pos, Y_XYZ_vel, svp, i);
      }
      gnss_Calc_B_XYZ(Y_XYZ_pos, Y_XYZ_vel, sp);//calculate coefficient matrix			
    }
    else if (!sp->fit_flag)
    {
      bds_sd_sv_pos_e(pBdsEph, t, svp, BDS_ID_IS_GEO(sv_id), &dClk, dopp_corr, freq_index);
      return dClk;
    }
    bds_sd_sv_clk_dop(pBdsEph, t, &dClk, dopp_corr, freq_index);
  }
  else if (gnssMode == GAL_MODE)
  {
    pGalEph = (GAL_EPH_INFO*)pEph;
    if (pGalEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    dt = t - sp->fit_tm_start;
    if (dt < -SECS_IN_WEEK / 2.0)
    {
      dt += (double)SECS_IN_WEEK;
    }

    if (fabs(dt) >= 60 || sp->toe != pGalEph->toe)
    {
      sp->fit_flag = 1;
      sp->fit_tm_start = t;
      sp->toe = (float)pGalEph->toe;
      dt = 0.0;

      for (i = 0; i < 4; i++)
      {
        if (!gal_sd_sv_pos_e(pGalEph, t + i * 20.0, svp, &dClk, dopp_corr, freq_index))
        {
          sp->fit_flag = 0;

          //check if need re-calculate
          if (i != 0)
          {
            gal_sd_sv_pos_e(pGalEph, t, svp, &dClk, dopp_corr, freq_index);
          }
          return dClk;
        }
        gnss_Copy_Y_XYZ(Y_XYZ_pos, Y_XYZ_vel, svp, i);
      }
      gnss_Calc_B_XYZ(Y_XYZ_pos, Y_XYZ_vel, sp);//calculate coefficient matrix				
    }
    else if (!sp->fit_flag)
    {
      gal_sd_sv_pos_e(pGalEph, t, svp, &dClk, dopp_corr, freq_index);
      return dClk;
    }
    gal_sd_sv_clk_dop(pGalEph, t, &dClk, dopp_corr, freq_index);
  }
  else
  {
    GLOGW("Wrong gnssMode in %s", __FUNCTION__);
  }

  gnss_Construct_R(dt, R);

  for (j = 0; j < 3; j++)
  {
    svp->p[j] = gnss_Multiply(R, &sp->B_XYZ_pos[4 * j], 4);
    svp->v[j] = (float)gnss_Multiply(R, &sp->B_XYZ_vel[4 * j], 4);
  }
  svp->t = t;
  svp->have_spos = FRM_EPH;

RET_PLACE:
  return dClk;
}

double gnss_Sd_Clk(uint8_t gnssMode, uint8_t sv_id, double t, double* dopp_corr, uint32_t freq_index)
{
  //uint8_t                i;
  //uint8_t                j;
  double               dt = 0.0;
  //double               R[4];
  //double				  Y_XYZ_pos[12];
  //double				  Y_XYZ_vel[12];
  double               dClk = 0.0;
  void* pEph;
  GPS_EPH_INFO* pGpsEph = NULL;
  GLN_EPH_INFO* pGlnEph = NULL;
  BDS_EPH_INFO* pBdsEph = NULL;
  GAL_EPH_INFO* pGalEph = NULL;

  /* Get the EPH */
  pEph = gnss_Sd_Nm_GetEph(gnssMode, sv_id);
  if (pEph == NULL)
  {
    GLOGI("There is no EPH for gnss(%02d) prn(%02d)", gnssMode, sv_id);
    return dClk;
  }

  /* Calculate satellite position and velocity */
  if (gnssMode == GPS_MODE)
  {
    pGpsEph = (GPS_EPH_INFO*)pEph;
    if (pGpsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    gps_sd_satpos_clk_dop(pGpsEph, t, &dClk, dopp_corr, freq_index);//calculate sat clock bias/drift using EPH
  }
  else if (gnssMode == GLN_MODE)
  {
    pGlnEph = (GLN_EPH_INFO*)pEph;
    if (pGlnEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    gln_sd_sv_clk_dop(1, pGlnEph, t, &dClk, dopp_corr);
  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsEph = (BDS_EPH_INFO*)pEph;
    if (pBdsEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    bds_sd_sv_clk_dop(pBdsEph, t, &dClk, dopp_corr, freq_index);
  }
  else if (gnssMode == GAL_MODE)
  {
    pGalEph = (GAL_EPH_INFO*)pEph;
    if (pGalEph->eph_status == EPH_STATUS_INVALID) goto RET_PLACE;

    gal_sd_sv_clk_dop(pGalEph, t, &dClk, dopp_corr, freq_index);
  }
  else
  {
    GLOGW("Wrong gnssMode in %s", __FUNCTION__);
  }

RET_PLACE:
  return dClk;
}
double gnss_Sd_Clk_Corr(uint8_t gnssMode, uint8_t sv_id, uint32_t freq_index)
{
  double               dClk = 0.0;
  void* pEph;
  GPS_EPH_INFO* pGpsEph = NULL;
  GLN_EPH_INFO* pGlnEph = NULL;
  BDS_EPH_INFO* pBdsEph = NULL;
  GAL_EPH_INFO* pGalEph = NULL;

  /* Get the EPH */
  pEph = gnss_Sd_Nm_GetEph(gnssMode, sv_id);
  if (pEph == NULL)
  {
    GLOGI("There is no EPH for gnssMode(%d) prn(%d)", gnssMode, sv_id);
    return dClk;
  }

  if (gnssMode == GPS_MODE)
  {
    pGpsEph = (GPS_EPH_INFO*)pEph;
    dClk += pGpsEph->subframe1.T_GD;
    if (freq_index == 1)
    {
      dClk -= pGpsEph->subframe1.T_GD * (GPS_L1_CARRIER / GPS_L2_CARRIER) * (GPS_L1_CARRIER / GPS_L2_CARRIER);
    }
  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsEph = (BDS_EPH_INFO*)pEph;
    dClk += pBdsEph->TGD1 * 1e-9;
    if (freq_index == 1)
    {
      dClk -= pBdsEph->TGD2 * 1e-9;
    }
  }
  else if (gnssMode == GAL_MODE)
  {
    pGalEph = (GAL_EPH_INFO*)pEph;
    dClk += pGalEph->bgd_e5b;
    if (!DBL_IS_EQUAL(pGalEph->bgd_e5a, 0.0) && freq_index == 2 && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_QCOM_855)//FNAV
    {
      dClk -= pGalEph->bgd_e5a * (GAL_E1_CARRIER / GAL_E5a_CARRIER) * (GAL_E1_CARRIER / GAL_E5a_CARRIER);  //for NAV/F in signal E5a
    }
    else
    {
      if (freq_index == 1)
      {
        dClk -= pGalEph->bgd_e5b * (GAL_E1_CARRIER / GAL_E5b_CARRIER) * (GAL_E1_CARRIER / GAL_E5b_CARRIER);  //for NAV/I in signal E5b
      }
    }
  }

  return dClk;
}

/**********************************************************************
* Function Name:    gps_sd_satpos_a
*
* Description:
*    gps_sd_satpos_a is function used to calculate GPS satellite position with ALM
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gps_sd_satpos_a(GPS_ALM_INFO* pAlm, double t, uint16_t wkn, ECEF* svp)
{
  float    tk;
  double    Mk, sin_v, cos_v, sin_w, cos_w, xkp, ykp, sinMk, cosMk, cosEk;
  double    b, b_prime, sinEk, r, cos_phik, sin_phik, sinok, cosok, sinik,
    cosik, ok, Ek;
  float    rdot, udot, vxkp, vykp;
  double    p0, p1;
  int16_t    dw;
  double    dbl_newTerm = (double)pAlm->e;

  tk = (float)t - pAlm->t_oa;
  /*  correct for different week. Watch week # wraparound! and week number roll-over */
  dw = (int16_t)(wkn - 256 * (int16_t)(wkn / 256)) - pAlm->wn_oa;

  if (dw < -128)
  {
    dw += 256;
  }
  else if (dw > 128)
  {
    dw -= 256;
  }

  // time check
  if (abs(dw) > ALM_USE_AGE)
  {
    GLOGI("Old ALM when calculating satellite position(%s)", __FUNCTION__);
    return;
  }

  if (dw)
  {
    tk += (float)dw * (float)604800.0;
  }

  // computer 
  pAlm->Axis = pAlm->sqrt_A * pAlm->sqrt_A;
  pAlm->n = (float)(WGS84_SQRT_U / ((double)pAlm->Axis * pAlm->sqrt_A));
  pAlm->OMEGA_n = (pAlm->OMEGA_0 - ((float)WGS84_OMEGDOTE * pAlm->t_oa));
  pAlm->ODOT_n = (pAlm->OMEGADOT - (float)WGS84_OMEGDOTE);
  pAlm->r1me2 = (float)sqrt((float)(1.0 - (double)pAlm->e * pAlm->e));
  pAlm->Axen = pAlm->Axis * pAlm->e * pAlm->n;


  Mk = (double)pAlm->M_0 + ((double)pAlm->n * tk);
  sinMk = sin(Mk);
  cosMk = cos(Mk);
  Ek = Mk + dbl_newTerm * sinMk * ((double)1.0 + dbl_newTerm * cosMk);
  sinEk = sin(Ek);
  cosEk = cos(Ek);
  b = (double)1.0 / ((double)1.0 - dbl_newTerm * cosEk);
  r = (double)pAlm->Axis / b;
  b_prime = (double)pAlm->r1me2 * b;
  sin_v = b_prime * sinEk;
  cos_v = (cosEk - dbl_newTerm) * b;
  sin_w = sin(pAlm->omega);
  cos_w = cos(pAlm->omega);
  cos_phik = cos_v * cos_w - sin_v * sin_w;
  sin_phik = sin_v * cos_w + cos_v * sin_w;
  xkp = r * cos_phik;
  ykp = r * sin_phik;
  ok = (double)pAlm->OMEGA_n + (double)pAlm->ODOT_n * tk;
  sinok = sin(ok);
  cosok = cos(ok);
  sinik = sin(pAlm->i_0);
  cosik = cos(pAlm->i_0);
  /* return float results as doubles... */
  svp->p[0] = xkp * cosok - ykp * (p0 = cosik * sinok);
  svp->p[1] = xkp * sinok + ykp * (p1 = cosik * cosok);
  svp->p[2] = ykp * sinik;
  svp->have_spos = FRM_ALM;
  /*  Compute velocity information based on Almanac */
  rdot = (float)((double)pAlm->Axen * sinEk * b);
  udot = (float)((double)pAlm->n * b_prime * b);
  vxkp = (float)((double)rdot * cos_phik - ykp * (double)udot);
  vykp = (float)((double)rdot * sin_phik + xkp * (double)udot);
  svp->v[0] = (float)((double)vxkp * cosok - svp->p[1] * (double)pAlm->ODOT_n - (double)vykp * p0);
  svp->v[1] = (float)((double)vxkp * sinok + svp->p[0] * (double)pAlm->ODOT_n + (double)vykp * p1);
  svp->v[2] = (float)((double)vykp * sinik);
}

/**********************************************************************
* Function Name:    gln_sd_satpos_a
*
* Description:
*    gln_sd_satpos_a is function used to calculate GLONASS satellite position with ALM
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gln_sd_satpos_a(GLN_ALM_INFO* pAlm, double t, uint16_t N4, uint16_t NT, ECEF* svp)
{
  uint8_t           loop;
  int16_t          N;

  double          i, TDR, a;
  double          lamda_dot;
  double          omiga_dot;
  double          dltat, n;
  double          M, E, lamda;
  double          omiga, EII;
  double          delta_T, temp;
  double          ae = 6378.136;//km
  double          u = 398600.44;//KM;
  double          Tmean = 43200;
  double          imean = 1.099557428756428;
  double          x0[3] = { 0 };
  double          e1[3] = { 0 };
  double          e2[3] = { 0 };
  double          x0_dot[2] = { 0 };

  N = (int16_t)(NT - pAlm->N_A);
  if ((NT - pAlm->N_A) > 800)       N = N - 1461;
  else if ((NT - pAlm->N_A) < -800) N = N + 1461;

  svp->have_spos = FRM_NULL;

  // time check
  if (abs(N) > 175)
  {
    GLOGI("Old ALM when calculating satellite position(%s)", __FUNCTION__);
    return;
  }

  i = imean + pAlm->DeltainA * PI;
  TDR = Tmean + pAlm->DeltaTnA; //delta_TA
  n = 2 * PI / TDR;
  a = pow((u / (n * n)), (1.0 / 3.0));
  lamda_dot = -2.020057004623067e-06 * cos(i) * pow(ae / a, 3.5);
  omiga_dot = 1.010028502311533e-06 * (5 * cos(i) * cos(i) - 1) * pow(ae / a, 3.5);
  dltat = t - pAlm->tLamdanA + 86400.0 * (N);
  lamda = pAlm->LamdanA * PI + (lamda_dot - OMGE_GLO) * dltat;
  omiga = pAlm->wnA * PI + omiga_dot * dltat;
  EII = 2 * atan(sqrt((1 - pAlm->EpsilonnA) / (1 + pAlm->EpsilonnA)) * tan(omiga / 2));
  delta_T = (EII - pAlm->EpsilonnA * sin(EII)) / n;

  if (omiga > PI)
  {
    delta_T = delta_T + TDR;
  }

  M = n * (dltat - delta_T);
  E = M;

  for (loop = 1; loop < 10; loop++)
  {
    temp = M + pAlm->EpsilonnA * sin(E);

    if (fabs(temp - E) < 1e-8)
    {
      E = temp;
      break;
    }

    E = temp;
  }

  x0[0] = a * (cos(E) - pAlm->EpsilonnA);
  x0[1] = a * sqrt(1.0 - pAlm->EpsilonnA * pAlm->EpsilonnA) * sin(E);
  e1[0] = cos(omiga) * cos(lamda) - sin(omiga) * sin(lamda) * cos(i);
  e1[1] = cos(omiga) * sin(lamda) + sin(omiga) * cos(lamda) * cos(i);
  e1[2] = sin(omiga) * sin(i);
  e2[0] = -sin(omiga) * cos(lamda) - cos(omiga) * sin(lamda) * cos(i);
  e2[1] = -sin(omiga) * sin(lamda) + cos(omiga) * cos(lamda) * cos(i);
  e2[2] = cos(omiga) * sin(i);
  x0_dot[0] = -a / (1 - pAlm->EpsilonnA * cos(E)) * n * sin(E);
  x0_dot[1] = a / (1 - pAlm->EpsilonnA * cos(E)) * n * sqrt(1.0 - pAlm->EpsilonnA * pAlm->EpsilonnA) * cos(E);
  svp->p[0] = (x0[0] * e1[0] + x0[1] * e2[0]) * 1000.0; // M in ECEF
  svp->p[1] = (x0[0] * e1[1] + x0[1] * e2[1]) * 1000.0; //M
  svp->p[2] = (x0[0] * e1[2] + x0[1] * e2[2]) * 1000.0; // M
  svp->v[0] = (float)((x0_dot[0] * e1[0] + x0_dot[1] * e2[0] + OMGE_GLO * (svp->p[1] / 1000.0)) * 1000.0); // M/s
  svp->v[1] = (float)((x0_dot[0] * e1[1] + x0_dot[1] * e2[1] + OMGE_GLO * (-svp->p[0] / 1000.0)) * 1000.0); //M/s
  svp->v[2] = (float)((x0_dot[0] * e1[2] + x0_dot[1] * e2[2]) * 1000.0); // m/s
  svp->have_spos = FRM_ALM;
  return;

}

/**********************************************************************
* Function Name:    bds_sd_satpos_a
*
* Description:
*    bds_sd_satpos_a is function used to calculate BDS satellite position with ALM
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void bds_sd_satpos_a(BDS_ALM_INFO* pAlm, double t, uint16_t week, ECEF* svp, uint8_t GEO)
{
  int16_t     dw;
  double     dt = 0;
  double     Mk, sin_v, cos_v, sin_w, cos_w, xkp, ykp, sinMk, cosMk, cosEk, n0, lt, p0, p1, b_prime;
  double     b, sinEk, r, cos_phik, sin_phik, sinok, cosok, sinik, cosik, ok, Ek, ik, i0, Axen;
  double     rdot, udot, vxkp, vykp;

  dt = t - pAlm->toa;
  dw = week % 256 - pAlm->WNa;

  if (dw < -128)
  {
    dw += 256;
  }
  else if (dw > 128)
  {
    dw -= 256;
  }
  if (abs(dw) > ALM_USE_AGE)
  {
    GLOGI("Old ALM when calculating satellite position(%s)", __FUNCTION__);
    return;
  }
  if (dw)
  {
    dt += (double)dw * (double)604800.0;
  }

  if (fabs(pAlm->sqrta) < (double)(1e-6) || fabs(pAlm->M0) < (double)1e-12 || fabs(pAlm->ecc) < (double)1e-12)
  {
    return;
  }

  n0 = (sqrt(MU_BDS) * pow(pAlm->sqrta, -3));
  lt = pAlm->omegaDot - OMGE_BDS;
  Axen = pAlm->sqrta * pAlm->sqrta * pAlm->ecc * n0;

  Mk = pAlm->M0 + n0 * dt;
  sinMk = sin(Mk);
  cosMk = cos(Mk);
  Ek = Mk + pAlm->ecc * sinMk * ((double)1.0 + pAlm->ecc * cosMk);
  sinEk = sin(Ek);
  cosEk = cos(Ek);
  b = (double)1.0 / ((double)1.0 - pAlm->ecc * cosEk);
  b_prime = sqrt(1.0 - (double)pAlm->ecc * pAlm->ecc) * b;
  r = pAlm->sqrta * pAlm->sqrta / b;
  sin_v = b_prime * sinEk;
  cos_v = (cosEk - pAlm->ecc) * b;
  sin_w = sin(pAlm->w);
  cos_w = cos(pAlm->w);
  cos_phik = cos_v * cos_w - sin_v * sin_w;
  sin_phik = sin_v * cos_w + cos_v * sin_w;
  xkp = r * cos_phik;
  ykp = r * sin_phik;
  ok = pAlm->omega0 + dt * lt - OMGE_BDS * pAlm->toa;
  sinok = sin(ok);
  cosok = cos(ok);

  if (GEO)
  {
    i0 = 0.0;
  }
  else
  {
    i0 = 0.3 * PI;
  }

  ik = i0 + pAlm->deltai;
  sinik = sin(ik);
  cosik = cos(ik);
  svp->p[0] = xkp * cosok - ykp * (p0 = cosik * sinok);
  svp->p[1] = xkp * sinok + ykp * (p1 = cosik * cosok);
  svp->p[2] = ykp * sinik;
  rdot = Axen * sinEk * b;
  udot = n0 * b_prime * b;
  vxkp = rdot * cos_phik - ykp * udot;
  vykp = rdot * sin_phik + xkp * udot;
  svp->v[0] = (float)(vxkp * cosok - svp->p[1] * lt - vykp * p0);
  svp->v[1] = (float)(vxkp * sinok + svp->p[0] * lt + vykp * p1);
  svp->v[2] = (float)(vykp * sinik);
  svp->have_spos = FRM_ALM;
}

/***********************************************************************
* 函数介绍: gnss_Sd_Pos_a
* 输入参数：gnssMode: 0 --> GPS, 1--> GLN, 2--> BDS
*           prn: PRN
*           t: tot, for GPS/BDS, it's second of week, for GLN, it's second of day
*           week: for GPS, it's GPS week number;for BDS, it's BDS week number
* 输出参数：svp: satellite position and velocity
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
uint8_t gnss_Sd_Pos_a(uint8_t gnssMode, uint8_t sv_id, uint16_t week, double t, ECEF* svp)
{
  uint16_t               N4 = 0, NT = 0;
  void* pAlm;
  double               t_gln = 0.0;
  GPS_ALM_INFO* pGpsAlm = NULL;
  GLN_ALM_INFO* pGlnAlm = NULL;
  BDS_ALM_INFO* pBdsAlm = NULL;
  GNSS_TIME* pTime;

  svp->have_spos = FRM_NULL;
  pAlm = gnss_Sd_Nm_GetAlm(gnssMode, sv_id);

  pTime = gnss_tm_get_time();

  if (pAlm == NULL)
  {
    GLOGI("There is no ALM for gnssMode(%d) prn(%d)", gnssMode, sv_id);
    return FALSE;
  }

  if (gnssMode == GPS_MODE)
  {
    pGpsAlm = (GPS_ALM_INFO*)pAlm;
    gps_sd_satpos_a(pGpsAlm, t, week, svp);

  }
  else if (gnssMode == GLN_MODE)              // For GLN, we need re-calculate tot,N4 and NT
  {
    pGlnAlm = (GLN_ALM_INFO*)pAlm;
    t_gln = pTime->rcvr_time[GLN_MODE];
    N4 = pTime->N4;
    NT = pTime->NT;

    t_gln -= 0.077;
    if (t_gln < 0.0)
    {
      t_gln += (double)SECS_IN_DAY;

      if (NT > 1)
      {
        NT -= 1;
      }
      else
      {
        NT = DAY_IN_FOUR_YEAR;
        N4 -= 1;
      }
    }

    gln_sd_satpos_a(pGlnAlm, t_gln, N4, NT, svp);

  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsAlm = (BDS_ALM_INFO*)pAlm;
    bds_sd_satpos_a(pBdsAlm, t, week, svp, BDS_ID_IS_GEO(sv_id));
  }
  else
  {
    GLOGW("Not Support Galileo");
  }
  return (svp->have_spos == FRM_ALM);
}

/***********************************************************************
* 函数介绍: gnss_Sd_Pos
* 输入参数：gnssMode: 0 --> GPS, 1--> GLN, 2--> BDS
*           prn: PRN
*           t: tot, for GPS/BDS, it's second of week, for GLN, it's second of day
*           week: for GPS, it's GPS week number;for BDS, it's BDS week number
* 输出参数：svp: satellite position and velocity
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
uint8_t gnss_Sd_Pos(uint8_t gnssMode, uint8_t sv_id, uint16_t week, double t, ECEF* svp, uint32_t freq_index)
{
  svp->have_spos = FRM_NULL;

  if (g_pe_cfg.ppk_mode)
  {
    gnss_Sd_Pos_e1(gnssMode, sv_id, t, svp, NULL, freq_index);
  }
  else
  {
    gnss_Sd_Pos_e(gnssMode, sv_id, t, svp, NULL, freq_index);// EPH only need tot
  }

  if (svp->have_spos == FRM_EPH)
  {
    return TRUE;
  }
  else
  {
    gnss_Sd_Pos_a(gnssMode, sv_id, week, t, svp);   // ALM need tot and week
    if (svp->have_spos == FRM_ALM)
    {
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
}
