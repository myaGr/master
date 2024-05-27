/**@file        tide_correction.c
 * @brief       tide correction
 * @details     solid earth tide, ocean tide loading, pole tide
 * @author      houxiaowei
 * @date        2022/05/27
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/5/27   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "tide_correction.h"
#include "mw_log.h"
#include <math.h>

static const double	H2_LOVE = 0.6078;
static const double	L2_LOVE = 0.0847;
static const double	H3_LOVE = 0.292;
static const double	L3_LOVE = 0.015;
static const double GME = 3.986004415E+14; /* earth gravitational constant */
static const double GMS = 1.327124E+20;    /* sun gravitational constant */
static const double GMM = 4.902801E+12;    /* moon gravitational constant */
static const double GM_SOLAR = 1.32712440e11;
static const double GM_EARTH = 398600.4415;
static const double GM_LUNA = 4902.7989;


/**
 * @brief Step 1: out-of-phase corrections induced by mantle inelasticity in the diurnal band
 * @param[in] xsun
 * @param[in] xmon
 * @param[in] rsun
 * @param[in] rmon
 * @param[in] Lnlt
 * @param[in] fac2mon
 * @param[in] fac2sun
 * @param[out] xcorsta
 */
static void tide_SolidStep1idiu(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta);

/**
 * @brief Step 1: out-of-phase corrections induced by mantle inelasticity in the semi-diurnal band
 * @param[in] xsun
 * @param[in] xmon
 * @param[in] rsun
 * @param[in] rmon
 * @param[in] Lnlt
 * @param[in] fac2mon
 * @param[in] fac2sun
 * @param[out] xcorsta
 */
static void tide_SolidStep1isem(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta);

/**
 * @brief Step 1: correction induced by the latitude dependence
 * @param xsun
 * @param xmon
 * @param rsun
 * @param rmon
 * @param Lnlt
 * @param fac2mon
 * @param fac2sun
 * @param xcorsta
 */
static void tide_SolidStep1l1(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta);

/**
 * @brief For the diurnal band corrections, (in-phase and out-of-phase frequency dependence):
 * @param t
 * @param fhr
 * @param xsta
 * @param Lnlt
 * @param xcorsta
 * note: they are called to account for the frequency dependence of the love numbers
 */
static void tide_SolidStep2diu(double t, double fhr, const double* xsta, LonAndLat_t Lnlt, double* xcorsta);

/**
 * @brief Corrections for the long-period band, (in-phase and out-of-phase frequency dependence):
 * @param t
 * @param Lnlt
 * @param xcorsta
 */
static void tide_SolidStep2lon(double t, LonAndLat_t Lnlt, double* xcorsta);


/**
 * @brief get earth rotation parameter values
 * @param[in] erp earth rotation parameters
 * @param[in] gpst
 * @param[out] erpv erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @return status (TRUE:ok, FALSE:error)
 */
BOOL tide_GetErp(const gnss_Erp_t* erp, const GpsTime_t* gpst, double* erpv)
{
  double mjd = 0.0;
  double day = 0.0; 
  double a = 0.0;
  int i = 0;
  int j = 0;
  int k = 0;

  if (erp->n <= 0) 
  {
    return FALSE;
  }

  mjd = tm_GetMJD(gpst);
  
  if (mjd <= erp->data[0].mjd) 
  {
    day = mjd - erp->data[0].mjd;
    erpv[0] = erp->data[0].xp + erp->data[0].xpr * day;
    erpv[1] = erp->data[0].yp + erp->data[0].ypr * day;
    erpv[2] = erp->data[0].ut1_utc - erp->data[0].lod * day;
    erpv[3] = erp->data[0].lod;
    return TRUE;
  }

  if (mjd >= erp->data[erp->n - 1].mjd) {
    day = mjd - erp->data[erp->n - 1].mjd;
    erpv[0] = erp->data[erp->n - 1].xp + erp->data[erp->n - 1].xpr * day;
    erpv[1] = erp->data[erp->n - 1].yp + erp->data[erp->n - 1].ypr * day;
    erpv[2] = erp->data[erp->n - 1].ut1_utc - erp->data[erp->n - 1].lod * day;
    erpv[3] = erp->data[erp->n - 1].lod;
    return 1;
  }

  for (j = 0, k = erp->n - 1; j < k - 1; ) 
  {
    i = (j + k) / 2;
    if (mjd < erp->data[i].mjd) 
    {
      k = i;
    }
    else 
    {
      j = i;
    }
  }
  
  if (erp->data[j].mjd == erp->data[j + 1].mjd) 
  {
    a = 0.5;
  }
  else 
  {
    a = (mjd - erp->data[j].mjd) / (erp->data[j + 1].mjd - erp->data[j].mjd);
  }
  erpv[0] = (1.0 - a) * erp->data[j].xp + a * erp->data[j + 1].xp;
  erpv[1] = (1.0 - a) * erp->data[j].yp + a * erp->data[j + 1].yp;
  erpv[2] = (1.0 - a) * erp->data[j].ut1_utc + a * erp->data[j + 1].ut1_utc;
  erpv[3] = (1.0 - a) * erp->data[j].lod + a * erp->data[j + 1].lod;
  return TRUE;
}

/* solar/lunar tides (ref [2] 7) ---------------------------------------------*/
void tide_SolarLunar(const double* eu, const double* rp, double GMp,
  const double* pos, double* dr)
{
  const double H3 = 0.292;
  const double L3 = 0.015;
  double r = 0.0;
  double ep[3] = { 0 };
  double latp = 0.0;
  double lonp = 0.0;
  double p = 0.0;
  double K2 = 0.0;
  double K3 = 0.0;
  double a = 0.0;
  double H2 = 0.0;
  double L2 = 0.0;
  double dp = 0.0;
  double du = 0.0;
  double cosp = 0.0;
  double sinl = 0.0;
  double cosl = 0.0;
  int i = 0;
  
  r = gnss_Norm(rp, 3);
  if (r <= 0.0) 
  {
    return;
  }

  for (i = 0; i < 3; i++) 
  {
    ep[i] = rp[i] / r;
  }

  K2 = GMp * SQR(RE_WGS84) * SQR(RE_WGS84) / (GME * r * r * r);
  K3 = K2 * RE_WGS84 / r;
  latp = asin(ep[2]); 
  lonp = atan2(ep[1], ep[0]);
  cosp = cos(latp); 
  sinl = sin(pos[0]); 
  cosl = cos(pos[0]);

  /* step1 in phase (degree 2) */
  p = (3.0 * sinl * sinl - 1.0) / 2.0;
  H2 = 0.6078 - 0.0006 * p;
  L2 = 0.0847 + 0.0002 * p;
  a = gnss_Dot(ep, eu, 3);
  dp = K2 * 3.0 * L2 * a;
  du = K2 * (H2 * (1.5 * a * a - 0.5) - 3.0 * L2 * a * a);

  /* step1 in phase (degree 3) */
  dp += K3 * L3 * (7.5 * a * a - 1.5);
  du += K3 * (H3 * (2.5 * a * a * a - 1.5 * a) - L3 * (7.5 * a * a - 1.5) * a);

  /* step1 out-of-phase (only radial) */
  du += 3.0 / 4.0 * 0.0025 * K2 * sin(2.0 * latp) * sin(2.0 * pos[0]) * sin(pos[1] - lonp);
  du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * sin(2.0 * (pos[1] - lonp));

  dr[0] = dp * ep[0] + du * eu[0];
  dr[1] = dp * ep[1] + du * eu[1];
  dr[2] = dp * ep[2] + du * eu[2];
  
  return;
}

/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
void tide_Solid(const double* rsun, const double* rmoon,
  const double* pos, const double* E, double gmst, int opt,
  double* dr)
{
  double dr1[3] = { 0.0 };
  double dr2[3] = { 0.0 };
  double eu[3] = { 0.0 };
  double du = 0.0;
  double dn = 0.0;
  double sinl = 0.0;
  double sin2l = 0.0;

  /* step1: time domain */
  eu[0] = E[2]; 
  eu[1] = E[5]; 
  eu[2] = E[8];
  tide_SolarLunar(eu, rsun, GMS, pos, dr1);
  tide_SolarLunar(eu, rmoon, GMM, pos, dr2);

  /* step2: frequency domain, only K1 radial */
  sin2l = sin(2.0 * pos[0]);
  du = -0.012 * sin2l * sin(gmst + pos[1]);

  dr[0] = dr1[0] + dr2[0] + du * E[2];
  dr[1] = dr1[1] + dr2[1] + du * E[5];
  dr[2] = dr1[2] + dr2[2] + du * E[8];

  /* eliminate permanent deformation */
  if (opt & 8) 
  {
    sinl = sin(pos[0]);
    du = 0.1196 * (1.5 * sinl * sinl - 0.5);
    dn = 0.0247 * sin2l;
    dr[0] += du * E[2] + dn * E[1];
    dr[1] += du * E[5] + dn * E[4];
    dr[2] += du * E[8] + dn * E[7];
  }
}

/* displacement by ocean tide loading (ref [2] 7) ----------------------------*/
void tide_Oceanload(const UtcTime_t* tut, const double* odisp, double* denu)
{
  UtcTime_t utc_base = { 0 };
  const double args[][5] = {
      {1.40519E-4, 2.0,-2.0, 0.0, 0.00},  /* M2 */
      {1.45444E-4, 0.0, 0.0, 0.0, 0.00},  /* S2 */
      {1.37880E-4, 2.0,-3.0, 1.0, 0.00},  /* N2 */
      {1.45842E-4, 2.0, 0.0, 0.0, 0.00},  /* K2 */
      {0.72921E-4, 1.0, 0.0, 0.0, 0.25},  /* K1 */
      {0.67598E-4, 1.0,-2.0, 0.0,-0.25},  /* O1 */
      {0.72523E-4,-1.0, 0.0, 0.0,-0.25},  /* P1 */
      {0.64959E-4, 1.0,-3.0, 1.0,-0.25},  /* Q1 */
      {0.53234E-5, 0.0, 2.0, 0.0, 0.00},  /* Mf */
      {0.26392E-5, 0.0, 1.0,-1.0, 0.00},  /* Mm */
      {0.03982E-5, 2.0, 0.0, 0.0, 0.00}   /* Ssa */
  };
  const EpochTime_t ep1975 = { 1975,1,1,0,0,0 };
  double fday, t, t2, t3, a[5], ang, dp[3] = { 0 };
  int days = 0;
  int i, j;

  // (ep1975) epoch->utc, diff, day and sec
  tm_cvt_EpochToUtcTime(&ep1975, &utc_base);
  double diff = tm_UtcTimeDiff(tut, &utc_base);
  days = (int)(diff / 86400) * 86400;
  fday = diff - days * 86400.0;

  /* angular argument: see subroutine arg.f for reference [1] */
  t = (27392.500528 + 1.000000035 * days) / 36525.0;
  t2 = t * t;
  t3 = t2 * t;

  a[0] = fday;
  a[1] = (279.69668 + 36000.768930485 * t + 3.03E-4 * t2) * DEG2RAD; /* H0 */
  a[2] = (270.434358 + 481267.88314137 * t - 0.001133 * t2 + 1.9E-6 * t3) * DEG2RAD; /* S0 */
  a[3] = (334.329653 + 4069.0340329577 * t - 0.010325 * t2 - 1.2E-5 * t3) * DEG2RAD; /* P0 */
  a[4] = 2.0 * PI;

  /* displacements by 11 constituents */
  for (i = 0; i < 11; i++) {
    ang = 0.0;
    for (j = 0; j < 5; j++) ang += a[j] * args[i][j];
    for (j = 0; j < 3; j++) dp[j] += odisp[j + i * 6] * cos(ang - odisp[j + 3 + i * 6] * DEG2RAD);
  }
  denu[0] = -dp[1];
  denu[1] = -dp[2];
  denu[2] = dp[0];
}


/* iers mean pole (ref [7] eq.7.25) ------------------------------------------*/
static void iers_mean_pole(const UtcTime_t* tut, double* xp_bar, double* yp_bar)
{
  UtcTime_t utc_base;
  const EpochTime_t ep = { 2000,1,1,0,0,0 };
  double y, y2, y3;

  tm_cvt_EpochToUtcTime(&ep, &utc_base);
  double diff = tm_UtcTimeDiff(tut, &utc_base);

  y = diff / 86400.0 / 365.25;

  if (y < 3653.0 / 365.25) { /* until 2010.0 */
    y2 = y * y; y3 = y2 * y;
    *xp_bar = 55.974 + 1.8243 * y + 0.18413 * y2 + 0.007024 * y3; /* (mas) */
    *yp_bar = 346.346 + 1.7896 * y - 0.10729 * y2 - 0.000908 * y3;
  }
  else { /* after 2010.0 */
    *xp_bar = 23.513 + 7.6141 * y; /* (mas) */
    *yp_bar = 358.891 - 0.6287 * y;
  }
}
/* displacement by pole tide (ref [7] eq.7.26) --------------------------------*/
extern void tide_Pole(const UtcTime_t* tut, const double* pos, const double* erpv,
  double* denu)
{
  double xp_bar, yp_bar, m1, m2, cosl, sinl;

  /* iers mean pole (mas) */
  iers_mean_pole(tut, &xp_bar, &yp_bar);

  /* ref [7] eq.7.24 */
  m1 = erpv[0] / ARCSEC2RAD - xp_bar * 1E-3; /* (as) */
  m2 = -erpv[1] / ARCSEC2RAD + yp_bar * 1E-3;

  /* sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi) */
  cosl = cos(pos[1]);
  sinl = sin(pos[1]);
  denu[0] = 9E-3 * sin(pos[0]) * (m1 * sinl - m2 * cosl); /* de= Slambda (m) */
  denu[1] = -9E-3 * cos(2.0 * pos[0]) * (m1 * cosl + m2 * sinl); /* dn=-Stheta  (m) */
  denu[2] = -33E-3 * sin(2.0 * pos[0]) * (m1 * cosl + m2 * sinl); /* du= Sr      (m) */

}
static void tide_pl(const double* eu, const double* rp, double GMp,
  const double* pos, double* dr)
{
  const double H3 = 0.292, L3 = 0.015;
  double r, ep[3], latp, lonp, p, K2, K3, a, H2, L2, dp, du, cosp, sinl, cosl;
  int i;

  if ((r = gnss_Norm(rp, 3)) <= 0.0) return;

  for (i = 0; i < 3; i++) ep[i] = rp[i] / r;

  K2 = GMp / GME * SQR(RE_WGS84) * SQR(RE_WGS84) / (r * r * r);
  K3 = K2 * RE_WGS84 / r;
  latp = asin(ep[2]); lonp = atan2(ep[1], ep[0]);
  cosp = cos(latp); sinl = sin(pos[0]); cosl = cos(pos[0]);

  /* step1 in phase (degree 2) */
  p = (3.0 * sinl * sinl - 1.0) / 2.0;
  H2 = 0.6078 - 0.0006 * p;
  L2 = 0.0847 + 0.0002 * p;
  a = gnss_Dot(ep, eu, 3);
  dp = K2 * 3.0 * L2 * a;
  du = K2 * (H2 * (1.5 * a * a - 0.5) - 3.0 * L2 * a * a);

  /* step1 in phase (degree 3) */
  dp += K3 * L3 * (7.5 * a * a - 1.5);
  du += K3 * (H3 * (2.5 * a * a * a - 1.5 * a) - L3 * (7.5 * a * a - 1.5) * a);

  /* step1 out-of-phase (only radial) */
  du += 3.0 / 4.0 * 0.0025 * K2 * sin(2.0 * latp) * sin(2.0 * pos[0]) * sin(pos[1] - lonp);
  du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * sin(2.0 * (pos[1] - lonp));

  dr[0] = dp * ep[0] + du * eu[0];
  dr[1] = dp * ep[1] + du * eu[1];
  dr[2] = dp * ep[2] + du * eu[2];
}
/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
static void tide_solid(const double* rsun, const double* rmoon,
  const double* pos, const double* E, double gmst, int opt,
  double* dr)
{
  double dr1[3]={0.0}, dr2[3]={0.0}, eu[3]={0.0}, du=0.0, dn=0.0, sinl=0.0, sin2l=0.0;

  /* step1: time domain */
  eu[0] = E[2]; eu[1] = E[5]; eu[2] = E[8];
  tide_pl(eu, rsun, GMS, pos, dr1);
  tide_pl(eu, rmoon, GMM, pos, dr2);

  /* step2: frequency domain, only K1 radial */
  sin2l = sin(2.0 * pos[0]);
  du = -0.012 * sin2l * sin(gmst + pos[1]);

  dr[0] = dr1[0] + dr2[0] + du * E[2];
  dr[1] = dr1[1] + dr2[1] + du * E[5];
  dr[2] = dr1[2] + dr2[2] + du * E[8];

  /* eliminate permanent deformation */
  if (opt & 8) {
    sinl = sin(pos[0]);
    du = 0.1196 * (1.5 * sinl * sinl - 0.5);
    dn = 0.0247 * sin2l;
    dr[0] += du * E[2] + dn * E[1];
    dr[1] += du * E[5] + dn * E[4];
    dr[2] += du * E[8] + dn * E[7];
  }
}

/**
 * @brief Tidal displacement
 * @param[in] gpst
 * @param[in] rr site position (ecef) (m)
 * @param[in] opt options (or of the followings)
 *                1: solid earth tide
 *                2: ocean tide loading
 *                4: pole tide
 *                8: elimate permanent deformation
 * @param[in] erp earth rotation parameters (NULL: not used)
 * @param[in] odisp ocean loading parameters  (NULL: not used)
 *                  odisp[0+i*6]: consituent i amplitude radial(m)
 *                  odisp[1+i*6]: consituent i amplitude west  (m)
 *                  odisp[2+i*6]: consituent i amplitude south (m)
 *                  odisp[3+i*6]: consituent i phase radial  (deg)
 *                  odisp[4+i*6]: consituent i phase west    (deg)
 *                  odisp[5+i*6]: consituent i phase south   (deg)
 *                  (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
 *                                   8:Mf,9:Mm,10:Ssa)
 * @param[out] dr displacement by earth tides (ecef) (m)
 * @return none
 * notes  : see ref [1], [2] chap 7
 *          see ref [4] 5.2.1, 5.2.2, 5.2.3
 *          ver.2.4.0 does not use ocean loading and pole tide corrections
 */
extern void tide_Displacement(const GpsTime_t* gpst, const double* rr, gnss_ssrErrorModelMask opt
  , const gnss_Erp_t* erp, const double* odisp, float* dr)
{
  UtcTime_t tut, utc;
  double pos[2], E[9], drt[3], denu[3], rs[3], rm[3], gmst, erpv[5] = { 0.0 };
  int i;
#ifdef IERS_MODEL
  double ep[6], fhr;
  int year, mon, day;
#endif

  if (NULL != erp)
  {
    tide_GetErp(erp, gpst, erpv);
  }
  // gpst->ut1
  tm_cvt_GpstToUtc(gpst, &utc);
  tut = utc;
  tm_UtcTimeAdd(&tut, erpv[2]);

  dr[0] = dr[1] = dr[2] = 0.0f;

  if (gnss_Norm(rr, 3) <= 0.0) return;

  pos[0] = asin(rr[2] / gnss_Norm(rr, 3));
  pos[1] = atan2(rr[1], rr[0]);
  gnss_Pos2EnuMat(pos, E);

  /* solid earth tides */
  if (!(opt & GNSS_SSR_ERROR_MODEL_SOLID_TIDE_CORR))
  {
    /* sun and moon position in ecef */
    gnss_SunMoonPos(gpst, erpv, rs, rm, &gmst);

    //tide_getSolidTide(gpst, pos, rs, rm, drt);
    tide_solid(rs, rm, pos, E, gmst, opt, drt);

    for (i = 0; i < 3; i++) dr[i] += (float)drt[i];
  }
  /* ocean tide loading */
  if (!(opt & GNSS_SSR_ERROR_MODEL_OCEAN_TID_CORR) && NULL!=odisp)
  {
    tide_Oceanload(&tut, odisp, denu);
    gnss_MatrixMultiply("TN", 3, 1, 3, 1.0, E, denu, 0.0, drt);
    for (i = 0; i < 3; i++) dr[i] += (float)drt[i];
  }
  /* pole tide */
  if (!(opt & GNSS_SSR_ERROR_MODEL_POLE_TIDE_CORR) && NULL != erp)
  {
    tide_Pole(&tut, pos, erpv, denu);
    gnss_MatrixMultiply("TN", 3, 1, 3, 1.0, E, denu, 0.0, drt);
    for (i = 0; i < 3; i++) dr[i] += (float)drt[i];
  }

  LOGD(TAG_PPP, "Earth tide: %.3lf %.3lf %.3lf\n", dr[0], dr[1], dr[2]);
}


/**
 * @brief Solid tide
 * @param[in] gpst
 * @param[in] recpos
 * @param[in] xsun
 * @param[in] xmoon
 * @param[out] corr
 * @return true or false
 */
extern bool tide_getSolidTide(const GpsTime_t* gpst, const double recpos[3], const double xsun[3],
  const double xmoon[3], double corr[3])
{
  if (fabs(xsun[0]) < 1.0e-2 || fabs(xsun[1]) < 1.0e-2 || fabs(xsun[2]) < 1.0e-2
    || fabs(xmoon[0]) < 1.0e-2 || fabs(xmoon[1]) < 1.0e-2 || fabs(xmoon[2]) < 1.0e-2)
  {
    return false;
  }
  double julian = tm_GetJD(gpst);
  double fhr = (julian - 0.5) - (int)(julian - 0.5);
  fhr = fhr * 24;
  if (fhr < 0.0)
  {
    fhr += 24.0;
  }
  double tdtutc = LEAP_SECS;
  double mass_ratio_sun = GM_SOLAR / GM_EARTH;
  double mass_ratio_moon = GM_LUNA / GM_EARTH;

  //-----------------------------------------------------------------------------------------------//
  double h2 = 0.0, l2 = 0.0;
  double p2sun = 0.0, p2mon = 0.0, p3sun = 0.0, p3mon = 0.0;
  double x2sun = 0.0, x2mon = 0.0, x3sun = 0.0, x3mon = 0.0;

  /* scalar product of station vector with sun/moon vector  */
  double rsta = sqrt(recpos[0] * recpos[0] + recpos[1] * recpos[1] + recpos[2] * recpos[2]);
  double rsun = sqrt(xsun[0] * xsun[0] + xsun[1] * xsun[1] + xsun[2] * xsun[2]);
  double rmon = sqrt(xmoon[0] * xmoon[0] + xmoon[1] * xmoon[1] + xmoon[2] * xmoon[2]);
  double scs = recpos[0] * xsun[0] + recpos[1] * xsun[1] + recpos[2] * xsun[2];
  double scm = recpos[0] * xmoon[0] + recpos[1] * xmoon[1] + recpos[2] * xmoon[2];

  double scsun = scs / rsta / rsun;
  double scmon = scm / rsta / rmon;
  LonAndLat_t Lnlt = { 0 };
  Lnlt._cosPhi = sqrt(recpos[0] * recpos[0] + recpos[1] * recpos[1]) / rsta;
  Lnlt._sinPhi = recpos[2] / rsta;
  Lnlt._cosTwoPhi = Lnlt._cosPhi * Lnlt._cosPhi - Lnlt._sinPhi * Lnlt._sinPhi;

  Lnlt._cosLa = recpos[0] / Lnlt._cosPhi / rsta;
  Lnlt._sinLa = recpos[1] / Lnlt._cosPhi / rsta;
  Lnlt._cosTwoLa = Lnlt._cosLa * Lnlt._cosLa - Lnlt._sinLa * Lnlt._sinLa;
  Lnlt._sinTwoLa = 2. * Lnlt._cosLa * Lnlt._sinLa;

  /* computation of new h2 and l2 */
  h2 = H2_LOVE - 0.0006 * (1. - 3. / 2. * (Lnlt._cosPhi * Lnlt._cosPhi));
  l2 = L2_LOVE + 0.0002 * (1. - 3. / 2. * (Lnlt._cosPhi * Lnlt._cosPhi));

  // p2-term  
  p2sun = 3. * (h2 / 2. - l2) * (scsun * scsun) - h2 / 2.;
  p2mon = 3. * (h2 / 2. - l2) * (scmon * scmon) - h2 / 2.;

  // p3-term  
  p3sun = 5. / 2. * (H3_LOVE - 3. * L3_LOVE) * (scsun * scsun * scsun)
    + 3. / 2. * (L3_LOVE - H3_LOVE) * scsun;
  p3mon = 5. / 2. * (H3_LOVE - 3. * L3_LOVE) * (scmon * scmon * scmon)
    + 3. / 2. * (L3_LOVE - H3_LOVE) * scmon;

  // term in direction of sun/moon vector  
  x2sun = 3. * l2 * scsun;
  x2mon = 3. * l2 * scmon;
  x3sun = 3. * L3_LOVE / 2. * (5. * (scsun * scsun) - 1.);
  x3mon = 3. * L3_LOVE / 2. * (5. * (scmon * scmon) - 1.);

  // factors for sun/moon  
  const double re = 6378136.55;
  double fac2sun = mass_ratio_sun * re * ((re * re * re) / (rsun * rsun * rsun));
  double fac2mon = mass_ratio_moon * re * ((re * re * re) / (rmon * rmon * rmon));
  double fac3sun = fac2sun * (re / rsun);
  double fac3mon = fac2mon * (re / rmon);

  // total displacement
  double dxtide[3] = { 0.0 };
  for (int i = 0; i < 3; ++i)
  {
    dxtide[i]
      = fac2sun * (x2sun * xsun[i] / rsun + p2sun * recpos[i] / rsta)
      + fac2mon * (x2mon * xmoon[i] / rmon + p2mon * recpos[i] / rsta)
      + fac3sun * (x3sun * xsun[i] / rsun + p3sun * recpos[i] / rsta)
      + fac3mon * (x3mon * xmoon[i] / rmon + p3mon * recpos[i] / rsta);
  }
  double xcorsta[3] = { 0.0 };

  tide_SolidStep1idiu(xsun, xmoon, rsun, rmon, Lnlt, fac2mon, fac2sun, xcorsta);

  for (size_t i = 0; i < 3; ++i)
  {
    dxtide[i] += xcorsta[i];
  }

  /* second, for the semi-diurnal band */
  tide_SolidStep1isem(xsun, xmoon, rsun, rmon, Lnlt, fac2mon, fac2sun, xcorsta);

  for (size_t i = 0; i < 3; ++i)
  {
    dxtide[i] += xcorsta[i];
  }

  /* corrections for the latitude dependence of love numbers (part l^(1)) */
  tide_SolidStep1l1(xsun, xmoon, rsun, rmon, Lnlt, fac2mon, fac2sun, xcorsta);

  for (size_t i = 0; i < 3; ++i)
  {
    dxtide[i] += xcorsta[i];
  }
  /* consider corrections for step 2
  *
  * corrections for the diurnal band:
  */
  /*  first, we need to know the date converted in julian centuries
  *
  *  1) call the subroutine computing the julian date
  *
  *     expression of the hours, minutes and secondes in fraction of day
  */
  //convert to centuries.

  double t = (julian - 2451545.0) / 36525.0;

  /*
  * 2) call the subroutine computing the correction of utc time
  */

  fhr += tdtutc / 3600.0;
  /*
  * second, we can call the subroutine step2diu, for the diurnal band corrections,
  * (in-phase and out-of-phase frequency dependence):
  */
  tide_SolidStep2diu(t, fhr, recpos, Lnlt, xcorsta);
  for (size_t i = 0; i < 3; ++i)
  {
    dxtide[i] += xcorsta[i];
  }

  /*
  *   corrections for the long-period band,
  *    (in-phase and out-of-phase frequency dependence):
  */
  tide_SolidStep2lon(t, Lnlt, xcorsta);

  for (size_t i = 0; i < 3; ++i)
  {
    dxtide[i] += xcorsta[i];
  }
  /*
  c consider corrections for step 3
  c
  c uncorrect for the permanent tide
  c
  c      pi=3.141592654
  c      Lnlt._sinPhi=xsta(3)/rsta
  c      Lnlt._cosPhi=dsqrt(xsta(1)**2+xsta(2)**2)/rsta
  c      Lnlt._cosLa=xsta(1)/Lnlt._cosPhi/rsta
  c      Lnlt._sinLa=xsta(2)/Lnlt._cosPhi/rsta
  c      dr=-dsqrt(5./4./pi)*h2*0.31460*(3./2.*Lnlt._sinPhi**2-0.5)
  c      dn=-dsqrt(5./4./pi)*l2*0.31460*3.*Lnlt._cosPhi*Lnlt._sinPhi
  c      dxtide(1)=dxtide(1)-dr*Lnlt._cosLa*Lnlt._cosPhi+dn*Lnlt._cosLa*Lnlt._sinPhi
  c      dxtide(2)=dxtide(2)-dr*Lnlt._sinLa*Lnlt._cosPhi+dn*Lnlt._sinLa*Lnlt._sinPhi
  c      dxtide(3)=dxtide(3)-dr*Lnlt._sinPhi      -dn*Lnlt._cosPhi
  */
  memcpy(corr, dxtide, 3 * sizeof(double));
  return true;
}

/**
 * @brief Step 1: out-of-phase corrections induced by mantle inelasticity in the diurnal band
 * @param[in] xsun
 * @param[in] xmon
 * @param[in] rsun
 * @param[in] rmon
 * @param[in] Lnlt
 * @param[in] fac2mon
 * @param[in] fac2sun
 * @param[out] xcorsta
 */
static void  tide_SolidStep1idiu(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta)
{
  double dhi = -0.0025;
  double dli = -0.0007;

  double drsun = 0.0, drmon = 0.0;
  double dnsun = 0.0, dnmon = 0.0;
  double desun = 0.0, demon = 0.0;
  double dr = 0.0, dn = 0.0, de = 0.0;

  drsun = -3. * dhi * Lnlt._sinPhi * Lnlt._cosPhi * fac2sun * xsun[2]
    * (xsun[0] * Lnlt._sinLa - xsun[1] * Lnlt._cosLa) / (rsun * rsun);
  drmon = -3. * dhi * Lnlt._sinPhi * Lnlt._cosPhi * fac2mon * xmon[2]
    * (xmon[0] * Lnlt._sinLa - xmon[1] * Lnlt._cosLa) / (rmon * rmon);
  dnsun = -3. * dli * Lnlt._cosTwoPhi * fac2sun * xsun[2]
    * (xsun[0] * Lnlt._sinLa - xsun[1] * Lnlt._cosLa) / (rsun * rsun);
  dnmon = -3. * dli * Lnlt._cosTwoPhi * fac2mon * xmon[2]
    * (xmon[0] * Lnlt._sinLa - xmon[1] * Lnlt._cosLa) / (rmon * rmon);
  desun = -3. * dli * Lnlt._sinPhi * fac2sun * xsun[2]
    * (xsun[0] * Lnlt._cosLa + xsun[1] * Lnlt._sinLa) / (rsun * rsun);
  demon = -3. * dli * Lnlt._sinPhi * fac2mon * xmon[2]
    * (xmon[0] * Lnlt._cosLa + xmon[1] * Lnlt._sinLa) / (rmon * rmon);

  dr = drsun + drmon;
  dn = dnsun + dnmon;
  de = desun + demon;

  xcorsta[0] = dr * Lnlt._cosLa * Lnlt._cosPhi - de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
  xcorsta[1] = dr * Lnlt._sinLa * Lnlt._cosPhi + de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
  xcorsta[2] = dr * Lnlt._sinPhi + dn * Lnlt._cosPhi;
  return;
}


/**
 * @brief Step 1: out-of-phase corrections induced by mantle inelasticity in the semi-diurnal band
 * @param[in] xsun
 * @param[in] xmon
 * @param[in] rsun
 * @param[in] rmon
 * @param[in] Lnlt
 * @param[in] fac2mon
 * @param[in] fac2sun
 * @param[out] xcorsta
 */
static void tide_SolidStep1isem(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta)
{
  double 	dhi = -0.0022;
  double	dli = -0.0007;

  double drsun = 0.0, drmon = 0.0;
  double dnsun = 0.0, dnmon = 0.0;
  double desun = 0.0, demon = 0.0;

  double dr = 0.0, dn = 0.0, de = 0.0;

  /* IN FORTRAN CODE : DATA DHI/-0.0022/,DLI/-0.0007/ */

  drsun = -3. / 4. * dhi * Lnlt._cosPhi * Lnlt._cosPhi * fac2sun
    * ((xsun[0] * xsun[0] - xsun[1] * xsun[1]) * Lnlt._sinTwoLa
      - 2. * xsun[0] * xsun[1] * Lnlt._cosTwoLa) / (rsun * rsun);

  drmon = -3. / 4. * dhi * Lnlt._cosPhi * Lnlt._cosPhi * fac2mon
    * ((xmon[0] * xmon[0] - xmon[1] * xmon[1]) * Lnlt._sinTwoLa
      - 2. * xmon[0] * xmon[1] * Lnlt._cosTwoLa) / (rmon * rmon);

  dnsun = 3. / 2. * dli * Lnlt._sinPhi * Lnlt._cosPhi * fac2sun
    * ((xsun[0] * xsun[0] - xsun[1] * xsun[1]) * Lnlt._sinTwoLa
      - 2. * xsun[0] * xsun[1] * Lnlt._cosTwoLa) / (rsun * rsun);

  dnmon = 3. / 2. * dli * Lnlt._sinPhi * Lnlt._cosPhi * fac2mon
    * ((xmon[0] * xmon[0] - xmon[1] * xmon[1]) * Lnlt._sinTwoLa
      - 2. * xmon[0] * xmon[1] * Lnlt._cosTwoLa) / (rmon * rmon);

  desun = -3. / 2. * dli * Lnlt._cosPhi * fac2sun
    * ((xsun[0] * xsun[0] - xsun[1] * xsun[1]) * Lnlt._cosTwoLa
      + 2. * xsun[0] * xsun[1] * Lnlt._sinTwoLa) / (rsun * rsun);

  demon = -3. / 2. * dli * Lnlt._cosPhi * fac2mon
    * ((xmon[0] * xmon[0] - xmon[1] * xmon[1]) * Lnlt._cosTwoLa
      + 2. * xmon[0] * xmon[1] * Lnlt._sinTwoLa) / (rmon * rmon);

  dr = drsun + drmon;
  dn = dnsun + dnmon;
  de = desun + demon;

  xcorsta[0] = dr * Lnlt._cosLa * Lnlt._cosPhi - de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
  xcorsta[1] = dr * Lnlt._sinLa * Lnlt._cosPhi + de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
  xcorsta[2] = dr * Lnlt._sinPhi + dn * Lnlt._cosPhi;
  return;
}

/**
 * @brief Step 1: correction induced by the latitude dependence
 * @param xsun
 * @param xmon
 * @param rsun
 * @param rmon
 * @param Lnlt
 * @param fac2mon
 * @param fac2sun
 * @param xcorsta
 */
static void tide_SolidStep1l1(const double* xsun, const double* xmon, double rsun,
  double rmon, LonAndLat_t Lnlt, double fac2mon, double fac2sun, double* xcorsta)
{
  double l1 = 0.0;
  double de = 0.0, dn = 0.0;
  double dnsun = 0.0, dnmon = 0.0;
  double desun = 0.0, demon = 0.0;

  //for the diurnal band  	
  l1 = 0.0012;
  dnsun = -l1 * Lnlt._sinPhi * Lnlt._sinPhi * fac2sun * xsun[2]
    * (xsun[0] * Lnlt._cosLa + xsun[1] * Lnlt._sinLa) / (rsun * rsun);
  dnmon = -l1 * Lnlt._sinPhi * Lnlt._sinPhi * fac2mon * xmon[2]
    * (xmon[0] * Lnlt._cosLa + xmon[1] * Lnlt._sinLa) / (rmon * rmon);

  desun = l1 * Lnlt._sinPhi * (Lnlt._cosPhi * Lnlt._cosPhi - Lnlt._sinPhi * Lnlt._sinPhi)
    * fac2sun * xsun[2]
    * (xsun[0] * Lnlt._sinLa - xsun[1] * Lnlt._cosLa) / (rsun * rsun);
  demon = l1 * Lnlt._sinPhi * (Lnlt._cosPhi * Lnlt._cosPhi - Lnlt._sinPhi * Lnlt._sinPhi)
    * fac2mon * xmon[2]
    * (xmon[0] * Lnlt._sinLa - xmon[1] * Lnlt._cosLa) / (rmon * rmon);
  de = 3. * (desun + demon);
  dn = 3. * (dnsun + dnmon);
  xcorsta[0] = -de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
  xcorsta[1] = de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
  xcorsta[2] = dn * Lnlt._cosPhi;

  // for the semi-diurnal band  

  l1 = 0.0024;

  dnsun = -l1 / 2. * Lnlt._sinPhi * Lnlt._cosPhi * fac2sun
    * ((xsun[0] * xsun[0] - xsun[1] * xsun[1]) * Lnlt._cosTwoLa
      + 2. * xsun[0] * xsun[1] * Lnlt._sinTwoLa) / (rsun * rsun);
  dnmon = -l1 / 2. * Lnlt._sinPhi * Lnlt._cosPhi * fac2mon
    * ((xmon[0] * xmon[0] - xmon[1] * xmon[1]) * Lnlt._cosTwoLa
      + 2. * xmon[0] * xmon[1] * Lnlt._sinTwoLa) / (rmon * rmon);
  desun = -l1 / 2. * Lnlt._sinPhi * Lnlt._sinPhi * Lnlt._cosPhi * fac2sun
    * ((xsun[0] * xsun[0] - xsun[1] * xsun[1]) * Lnlt._sinTwoLa
      - 2. * xsun[0] * xsun[1] * Lnlt._cosTwoLa) / (rsun * rsun);
  demon = -l1 / 2. * Lnlt._sinPhi * Lnlt._sinPhi * Lnlt._cosPhi * fac2mon
    * ((xmon[0] * xmon[0] - xmon[1] * xmon[1]) * Lnlt._sinTwoLa
      - 2. * xmon[0] * xmon[1] * Lnlt._cosTwoLa) / (rmon * rmon);

  de = 3. * (desun + demon);
  dn = 3. * (dnsun + dnmon);
  xcorsta[0] = xcorsta[0] - de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
  xcorsta[1] = xcorsta[1] + de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
  xcorsta[2] = xcorsta[2] + dn * Lnlt._cosPhi;
  return;
}


/**
 * @brief For the diurnal band corrections, (in-phase and out-of-phase frequency dependence):
 * @param t
 * @param fhr
 * @param xsta
 * @param Lnlt
 * @param xcorsta
 * note: they are called to account for the frequency dependence of the love numbers
 */
static void tide_SolidStep2diu(double t, double fhr, const double* xsta, LonAndLat_t Lnlt, double* xcorsta)
{
  int i = 0, j = 0;
  double s = 0.0;
  double tau = 0.0, pr = 0.0, h = 0.0, p = 0.0, zns = 0.0, ps = 0.0;
  double zla = 0.0, thetaf = 0.0, dr = 0.0, dn = 0.0, de = 0.0;
  double t2 = 0.0, t3 = 0.0, t4 = 0.0;
  double deg2rad = 0.0174532925;
  static const double datdi[31][9] = {
          {-3., 0., 2., 0., 0., -0.01, 0.0, 0.0, 0.0,},
          {-3., 2., 0., 0., 0., -0.01, 0.0, 0.0, 0.0,},
          {-2., 0., 1., -1., 0., -0.02, 0.0, 0.0, 0.0,},
          {-2., 0., 1., 0., 0., -0.08, 0.0, -0.01, 0.01,},
          {-2., 2., -1., 0., 0., -0.02, 0.0, 0.0, 0.0,},
          {-1., 0., 0., -1., 0., -0.10, 0.0, 0.0, 0.0,},
          {-1., 0., 0., 0., 0., -0.51, 0.0, -0.02, 0.03,},
          {-1., 2., 0., 0., 0., 0.01, 0.0, 0.0, 0.0,},
          {0., -2., 1., 0., 0., 0.01, 0.0, 0.0, 0.0,},
          {0., 0., -1., 0., 0., 0.02, 0.0, 0.0, 0.0,},
          {0., 0., 1., 0., 0., 0.06, 0.0, 0.0, 0.0,},
          {0., 0., 1., 1., 0., 0.01, 0.0, 0.0, 0.0,},
          {0., 2., -1., 0., 0., 0.01, 0.0, 0.0, 0.0,},
          {1., -3., 0., 0., 1., -0.06, 0.0, 0.0, 0.0,},
          {1., -2., 0., -1., 0., 0.01, 0.0, 0.0, 0.0,},
          {1., -2., 0., 0., 0., -1.23, -0.07, 0.06, 0.01,},
          {1., -1., 0., 0., -1., 0.02, 0.0, 0.0, 0.0,},
          {1., -1., 0., 0., 1., 0.04, 0.0, 0.0, 0.0,},
          {1., 0., 0., -1., 0., -0.22, 0.01, 0.01, 0.0,},
          {1., 0., 0., 0., 0., 12.00, -0.80, -0.67, -0.03,},
          {1., 0., 0., 1., 0., 1.73, -0.12, -0.10, 0.0,},
          {1., 0., 0., 2., 0., -0.04, 0.0, 0.0, 0.0,},
          {1., 1., 0., 0., -1., -0.50, -0.01, 0.03, 0.0,},
          {1., 1., 0., 0., 1., 0.01, 0.0, 0.0, 0.0,},
          {0., 1., 0., 1., -1., -0.01, 0.0, 0.0, 0.0,},
          {1., 2., -2., 0., 0., -0.01, 0.0, 0.0, 0.0,},
          {1., 2., 0., 0., 0., -0.11, 0.01, 0.01, 0.0,},
          {2., -2., 1., 0., 0., -0.01, 0.0, 0.0, 0.0,},
          {2., 0., -1., 0., 0., -0.02, 0.0, 0.0, 0.0,},
          {3., 0., 0., 0., 0., 0.0, 0.0, 0.0, 0.0,},
          {3., 0., 0., 1., 0., 0.0, 0.0, 0.0, 0.0}
  };

  t2 = t * t;
  t3 = t2 * t;
  t4 = t2 * t2;

  s = 218.31664563 + 481267.88194 * t - 0.0014663889 * t2
    + 0.00000185139 * t3;
  tau = fhr * 15. + 280.4606184 + 36000.7700536 * t + 0.00038793 * t2
    - 0.0000000258 * t3 - s;
  pr = 1.396971278 * t + 0.000308889 * t2 + 0.000000021 * t3
    + 0.000000007 * t4;
  s = s + pr;
  h = 280.46645 + 36000.7697489 * t + 0.00030322222 * t2
    + 0.000000020 * t3 - 0.00000000654 * t4;
  p = 83.35324312 + 4069.01363525 * t - 0.01032172222 * t2
    - 0.0000124991 * t3 + 0.00000005263 * t4;
  zns = 234.95544499 + 1934.13626197 * t - 0.00207561111 * t2
    - 0.00000213944 * t3 + 0.00000001650 * t4;
  ps = 282.93734098 + 1.71945766667 * t + 0.00045688889 * t2
    - 0.00000001778 * t3 - 0.00000000334 * t4;

  // reduce angles to between 0 and 360.
  s = fmod(s, 360.);
  tau = fmod(tau, 360.);
  h = fmod(h, 360.);
  p = fmod(p, 360.);
  zns = fmod(zns, 360.);
  ps = fmod(ps, 360.);

  zla = atan2(xsta[1], xsta[0]);

  xcorsta[0] = xcorsta[1] = xcorsta[2] = 0.;

  for (j = 0; j < 31; ++j)
  {
    thetaf = (tau + datdi[j][0] * s + datdi[j][1] * h + datdi[j][2] * p
      + datdi[j][3] * zns + datdi[j][4] * ps) * deg2rad;
    dr = datdi[j][5] * 2. * Lnlt._sinPhi * Lnlt._cosPhi * sin(thetaf + zla)
      + datdi[j][6] * 2. * Lnlt._sinPhi * Lnlt._cosPhi * cos(thetaf + zla);
    dn = datdi[j][7] * (Lnlt._cosPhi * Lnlt._cosPhi - Lnlt._sinPhi * Lnlt._sinPhi)
      * sin(thetaf + zla)
      + datdi[j][8] * (Lnlt._cosPhi * Lnlt._cosPhi - Lnlt._sinPhi * Lnlt._sinPhi)
      * cos(thetaf + zla);
    de = datdi[j][7] * Lnlt._sinPhi * cos(thetaf + zla)
      + datdi[j][8] * Lnlt._sinPhi * sin(thetaf + zla);
    xcorsta[0] += dr * Lnlt._cosLa * Lnlt._cosPhi - de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
    xcorsta[1] += dr * Lnlt._sinLa * Lnlt._cosPhi + de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
    xcorsta[2] += dr * Lnlt._sinPhi + dn * Lnlt._cosPhi;
  }

  for (i = 0; i < 3; ++i)
  {
    xcorsta[i] /= 1000.;
  }
  return;
}

/**
 * @brief Corrections for the long-period band, (in-phase and out-of-phase frequency dependence):
 * @param t
 * @param Lnlt
 * @param xcorsta
 */
static void tide_SolidStep2lon(double t, LonAndLat_t Lnlt, double* xcorsta)
{
  int i = 0, j = 0;
  double s = 0.0, pr = 0.0, h = 0.0, p = 0.0, zns = 0.0, ps = 0.0;
  double dr_tot = 0.0, dn_tot = 0.0, thetaf = 0.0;
  double dr = 0.0, dn = 0.0, de = 0.0;
  double t2 = 0.0, t3 = 0.0, t4 = 0.0;
  double deg2rad = 0.0174532925;
  const double datdi[5][9] = {
          {0, 0, 0,  1, 0, 0.47,  0.23,  0.16,  0.07,},
          {0, 2, 0,  0, 0, -0.20, -0.12, -0.11, -0.05,},
          {1, 0, -1, 0, 0, -0.11, -0.08, -0.09, -0.04,},
          {2, 0, 0,  0, 0, -0.13, -0.11, -0.15, -0.07,},
          {2, 0, 0,  1, 0, -0.05, -0.05, -0.06, -0.03},
  };

  t2 = t * t;
  t3 = t2 * t;
  t4 = t2 * t2;

  s = 218.31664563 + 481267.88194 * t - 0.0014663889 * t2
    + 0.00000185139 * t3;
  pr = 1.396971278 * t + 0.000308889 * t2 + 0.000000021 * t3
    + 0.000000007 * t4;
  s = s + pr;
  h = 280.46645 + 36000.7697489 * t + 0.00030322222 * t2
    + 0.000000020 * t3 - 0.00000000654 * t4;
  p = 83.35324312 + 4069.01363525 * t - 0.01032172222 * t2
    - 0.0000124991 * t3 + 0.00000005263 * t4;
  zns = 234.95544499 + 1934.13626197 * t - 0.00207561111 * t2
    - 0.00000213944 * t3 + 0.00000001650 * t4;
  ps = 282.93734098 + 1.71945766667 * t + 0.00045688889 * t2
    - 0.00000001778 * t3 - 0.00000000334 * t4;

  // reduce angles to between 0 and 360.
  s = fmod(s, 360.);
  h = fmod(h, 360.);
  p = fmod(p, 360.);
  zns = fmod(zns, 360.);
  ps = fmod(ps, 360.);

  dr_tot = 0.;
  dn_tot = 0.;

  xcorsta[0] = xcorsta[1] = xcorsta[2] = 0.;

  for (j = 0; j < 5; ++j)
  {
    thetaf = (datdi[j][0] * s + datdi[j][1] * h
      + datdi[j][2] * p + datdi[j][3] * zns
      + datdi[j][4] * ps) * deg2rad;
    dr = datdi[j][5] * (3. * Lnlt._sinPhi * Lnlt._sinPhi - 1.) / 2. * cos(thetaf)
      + datdi[j][7] * (3. * Lnlt._sinPhi * Lnlt._sinPhi - 1.) / 2. * sin(thetaf);
    dn = datdi[j][6] * (Lnlt._cosPhi * Lnlt._sinPhi * 2.) * cos(thetaf)
      + datdi[j][8] * (Lnlt._cosPhi * Lnlt._sinPhi * 2.) * sin(thetaf);
    de = 0.;
    dr_tot += dr;
    dn_tot += dn;
    xcorsta[0] += dr * Lnlt._cosLa * Lnlt._cosPhi - de * Lnlt._sinLa - dn * Lnlt._sinPhi * Lnlt._cosLa;
    xcorsta[1] += dr * Lnlt._sinLa * Lnlt._cosPhi + de * Lnlt._cosLa - dn * Lnlt._sinPhi * Lnlt._sinLa;
    xcorsta[2] += dr * Lnlt._sinPhi + dn * Lnlt._cosPhi;
  }

  for (i = 0; i < 3; ++i)
  {
    xcorsta[i] /= 1000.;
  }
  return;
}