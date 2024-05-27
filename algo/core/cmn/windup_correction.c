/**@file        windup_correction.c
 * @brief       satellite windup correction
 * @details     phase windup effect
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

#include "windup_correction.h"
#include <math.h>
#include "mw_alloc.h"
#include "sat_attitude.h"
#include "gnss_def.h"
#include "gnss_common.h"


static SatAttitude_t* gpz_sat_attitude_set[ALL_GNSS_SYS_SV_NUMBER]= {0};

/**
 * @brief deinit windup correction
 */
extern void wup_deinit()
{
  for (uint16_t w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    OS_FREE(gpz_sat_attitude_set[w_i]);
  }
}

/**
 * @brief instance of sat attitude struct
 * @param[i] sat
 * @return SatAttitude_t
 */
static SatAttitude_t* getSatAttitudeInstance(uint8_t sat, uint8_t sys)
{
  uint8_t idx = gnss_cvt_Svid2SvIndex(sat, sys);
  if(gpz_sat_attitude_set[idx] == NULL)
  {
    gpz_sat_attitude_set[idx] = (SatAttitude_t*)OS_MALLOC(sizeof(SatAttitude_t));
  }
  return gpz_sat_attitude_set[idx];
}

/**
 * @brief windup correction
 * @param[in] svid
 * @param[in] sys
 * @param[in] yaw
 * @param[in] gpst
 * @param[in] rs
 * @param[in] vs
 * @param[in] rr
 * @param[out] wind_up_corr
 * @return
 */
extern bool wup_getWindUp(const GpsTime_t *gpst, uint8_t svid, uint8_t sys, const double *yaw, const double rs[3],
                   const double vs[3], const double rr[3], double *wind_up_corr)
{
  static float _wind_up[ALL_GNSS_SYS_SV_NUMBER] = {.0};
  double sat2rec_unit[3] = {0.0};
  double blh[3] = {.0};
  double d_windup_r[3] = {0.0};
  double dummy[3] = {0.0};
  double d_windup_s[3] = {0.0};

  *wind_up_corr = 0.0;

  for (int i = 0; i < 3; i++) {
    sat2rec_unit[i] = rr[i] - rs[i];
  }
  gnss_UnitVector(sat2rec_unit, sat2rec_unit, 3);
  gnss_Ecef2Lla(rr, blh);

  /* for receiver */
  wup_ReceiverWindUp(sat2rec_unit, blh, d_windup_r);

  /* for satellite */
  if (wup_SatWindUp(svid, sys, gpst, yaw, rs, vs, sat2rec_unit, d_windup_s) == false) {
    return false;
  }

  /* calculation wind up */
  gnss_CrossMultiply(d_windup_s, d_windup_r, dummy);
  double sign = gnss_Dot(sat2rec_unit, dummy, 3);
  double dot_reslut = gnss_Dot(d_windup_s, d_windup_r, 3);
  if (dot_reslut < -1.0) {
    dot_reslut = -1.0;
  }
  else if (dot_reslut > 1.0) {
    dot_reslut = 1.0;
  }
  if (sign < 0.0) {
    *wind_up_corr = -acos(dot_reslut) / (2 * PI);
  }
  else {
    *wind_up_corr = acos(dot_reslut) / (2 * PI);
  }
  int index = gnss_cvt_Svid2SvIndex(svid, sys);
  if (index == ALL_GNSS_SYS_SV_NUMBER) {
    return false;
  }
  double n = gnss_Round(_wind_up[index] - *wind_up_corr, 1);
  *wind_up_corr += n;
  _wind_up[index] = (float)(*wind_up_corr);
  return true;
}

/**
 * @brief receiver windup
 * @param[in] sat2rec_unit
 * @param[in] blh
 * @param[out] d_windup_r
 */
extern void wup_ReceiverWindUp(const double sat2rec_unit[3], const double blh[3], double d_windup_r[3])
{
  double dummy[3] = {.0};
  double coslat = cos(blh[0] - PI / 2.0);
  double sinlat = sin(blh[0] - PI / 2.0);
  double coslon = cos(-PI / 2.0 - blh[1]);
  double sinlon = sin(-PI / 2.0 - blh[1]);

  double Tx[9] = {0.0};
  Tx[0] = coslon;
  Tx[1] = sinlon * coslat;
  Tx[2] = sinlon * sinlat;
  Tx[3] = -sinlon;
  Tx[4] = coslon * coslat;
  Tx[5] = coslon * sinlat;
  Tx[6] = 0.0;
  Tx[7] = -sinlat;
  Tx[8] = coslat;
  double x_l[3] = {0.0, 1.0, 0.0};
  double exr[3] = {0.0};    /* x = north */
  gnss_MatrixMultiply("NN", 3, 3, 1, 1.0, Tx, x_l, 0.0, exr);

  double y_l[3] = {-1.0, 0.0, 0.0};
  double eyr[3] = {0.0};    /* y = east  */
  gnss_MatrixMultiply("NN", 3, 3, 1, 1.0, Tx, y_l, 0.0, eyr);

  double dist = gnss_Dot(sat2rec_unit, exr, 3);
  gnss_CrossMultiply(sat2rec_unit, eyr, dummy);

  for (size_t i = 0; i < 3; i++) {
    d_windup_r[i] = exr[i] - dist * sat2rec_unit[i] + dummy[i];
  }
  gnss_UnitVector(d_windup_r, d_windup_r, 3);
}

/**
 * @brief satellite windup
 * @param[in] svid
 * @param[in] sys
 * @param[in] gpst
 * @param[in] yaw
 * @param[in] rs
 * @param[in] vs
 * @param[in] sat2rec_unit
 * @param[out] d_windup_s
 * @return
 */
extern bool wup_SatWindUp(int svid, int sys, const GpsTime_t *gpst, const double *yaw, const double rs[3], const double vs[3],
                   const double sat2rec_unit[3], double d_windup_s[3])
{
  double x_point[3] = {.0}, y_point[3] = {0}, z_point[3] = {.0}, dummy[3] = {.0};

  SatAttitude_t* sat_atude = getSatAttitudeInstance(svid, sys);
  if(NULL==sat_atude)
  {
    return false;
  }
  at_IntputSatInfo(sat_atude, gpst, svid, sys, rs, vs);

  if (false == at_GetAttitude(sat_atude, yaw, x_point, y_point, z_point)) {
    return false;
  }
  double dist = gnss_Dot(sat2rec_unit, x_point, 3);
  gnss_CrossMultiply(sat2rec_unit, y_point, dummy);

  for (size_t i = 0; i < 3; i++) {
    d_windup_s[i] = x_point[i] - dist * sat2rec_unit[i] - dummy[i];
  }
  gnss_UnitVector(d_windup_s, d_windup_s, 3);

  return true;
}

/*********** RTKLIB ************/

/* nominal yaw-angle ---------------------------------------------------------*/
static double yaw_nominal(double beta, double mu)
{
  if (fabs(beta)<1E-12&&fabs(mu)<1E-12) return PI;
  return atan2(-tan(beta),sin(mu))+PI;
}
/* yaw-angle of satellite ----------------------------------------------------*/
static int yaw_angle(int sat, const char *type, int opt, double beta, double mu,
        double *yaw)
{
  *yaw=yaw_nominal(beta,mu);
  return 1;
}
/* satellite attitude model --------------------------------------------------*/
static int sat_yaw(const GpsTime_t* time, int sat, const char *type, int opt,
        const double *rs, double *exs, double *eys)
{
  double rsun[3],ri[6],es[3],esun[3],n[3],p[3],en[3],ep[3],ex[3],E,beta,mu;
  double yaw,cosy,siny,erpv[5]={0};
  int i;

  gnss_SunMoonPos(time,erpv,rsun,NULL,NULL);

  /* beta and orbit angle */
  memcpy(ri, rs, sizeof (double)*6);
  ri[3]-=OMGE_GPS*ri[1];
  ri[4]+=OMGE_GPS*ri[0];
  gnss_CrossMultiply(ri,ri+3,n);
  gnss_CrossMultiply(rsun,n,p);
  if (!gnss_UnitVector(rs,es,3)||!gnss_UnitVector(rsun,esun,3)||!gnss_UnitVector(n,en,3)||
      !gnss_UnitVector(p,ep,3)) return 0;
  beta=PI/2.0-acos(gnss_Dot(esun,en,3));
  E=acos(gnss_Dot(es,ep,3));
  mu=PI/2.0+(gnss_Dot(es,esun,3)<=0?-E:E);
  if      (mu<-PI/2.0) mu+=2.0*PI;
  else if (mu>=PI/2.0) mu-=2.0*PI;

  /* yaw-angle of satellite */
  yaw_angle(sat,type,opt,beta,mu,&yaw);

  /* satellite fixed x,y-vector */
  gnss_CrossMultiply(en,es,ex);
  cosy=cos(yaw);
  siny=sin(yaw);
  for (i=0;i<3;i++) {
    exs[i]=-siny*en[i]+cosy*ex[i];
    eys[i]=-cosy*en[i]-siny*ex[i];
  }
  return 1;
}

/* phase windup model --------------------------------------------------------*/
extern int model_phw(const GpsTime_t* time, int sat, const char* type, int opt, const double* rs, const double* rr, double* phw)
{
  static float phw_list[ALL_GNSS_SYS_SV_NUMBER] = {0.0f};
  double exs[3],eys[3],ek[3],exr[3],eyr[3],eks[3],ekr[3],E[9];
  double dr[3],ds[3],drs[3],r[3],pos[3],cosp,ph;
  int i;

  *phw = phw_list[sat-1];
  if (opt<=0) return 0; /* no phase windup */

  /* satellite yaw attitude model */
  if (!sat_yaw(time,sat,type,opt,rs,exs,eys)) return 0;

  /* unit vector satellite to receiver */
  for (i=0;i<3;i++) r[i]=rr[i]-rs[i];
  if (!gnss_UnitVector(r,ek,3)) return 0;

  /* unit vectors of receiver antenna */
  gnss_Ecef2Lla(rr,pos);
  gnss_Pos2EnuMat(pos,E);
  exr[0]= E[1]; exr[1]= E[4]; exr[2]= E[7]; /* x = north */
  eyr[0]=-E[0]; eyr[1]=-E[3]; eyr[2]=-E[6]; /* y = west  */

  /* phase windup effect */
  gnss_CrossMultiply(ek,eys,eks);
  gnss_CrossMultiply(ek,eyr,ekr);
  for (i=0;i<3;i++) {
    ds[i]=exs[i]-ek[i]*gnss_Dot(ek,exs,3)-eks[i];
    dr[i]=exr[i]-ek[i]*gnss_Dot(ek,exr,3)+ekr[i];
  }
  cosp=gnss_Dot(ds,dr,3)/gnss_Norm(ds,3)/gnss_Norm(dr,3);
  if      (cosp<-1.0) cosp=-1.0;
  else if (cosp> 1.0) cosp= 1.0;
  ph=acos(cosp)/2.0/PI;
  gnss_CrossMultiply(ds,dr,drs);
  if (gnss_Dot(ek,drs,3)<0.0) ph=-ph;

  *phw=ph+floor(*phw-ph+0.5); /* in cycle */
  phw_list[sat-1] = (float)(*phw);
  return 1;
 }