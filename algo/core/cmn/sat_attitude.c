/**@file        sat_attitude.c
 * @brief       satellite attitude
 * @details     GNSS satellite attitude control mode
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

#include "sat_attitude.h"
#include <math.h>
#include "gnss_common.h"


const double ECLIP_YRATE_GPS[32] =
        // PRN      01      02      03      04      05      06      07      08
        {0.1211, 0.1339, 0.1230, 0.1233, 0.1180, 0.1266, 0.1269, 0.1033,
                // PRN      09      10      11      12      13      14      15      16
         0.1278, 0.0978, 0.2000, 0.1990, 0.2000, 0.0815, 0.1303, 0.0838,
                // PRN      17      18      19      20      21      22      23      24
         0.1401, 0.1069, 0.0980, 0.1030, 0.1366, 0.1025, 0.1140, 0.1089,
                // PRN      25      26      27      28      29      30      31      32
         0.1001, 0.1227, 0.1194, 0.1260, 0.1228, 0.1165, 0.0969, 0.1140};
static const double TRUNC_ERR = 1.0e-12;
static const double BDS2_BETA_MAX = 4.0;
static const double BDS2_YAW_MAX = 5.0;


static const uint8_t GPS_SAT_TYPE_SET[MAX_SAT_TYPE_NUM_GPS] =
        {BLOCK_IIF, BLOCK_IIRB, BLOCK_IIF, BLOCK_IIIA, BLOCK_IIRM, BLOCK_IIF, BLOCK_IIRM, BLOCK_IIF,
         BLOCK_IIF, BLOCK_IIF, BLOCK_IIRA, BLOCK_IIRM, BLOCK_IIRA, BLOCK_IIIA, BLOCK_IIRM, BLOCK_IIRA,
         BLOCK_IIRM, BLOCK_IIIA, BLOCK_IIRB, BLOCK_IIRA, BLOCK_IIRA, BLOCK_IIRB, BLOCK_IIIA, BLOCK_IIF,
         BLOCK_IIF, BLOCK_IIF, BLOCK_IIF, BLOCK_IIRA, BLOCK_IIRM, BLOCK_IIF, BLOCK_IIRM, BLOCK_IIF, INVALID_SAT_TYPE};

static const uint8_t GLO_SAT_TYPE_SET[MAX_SAT_TYPE_NUM_GLO] =
        {GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M,
         GLO_K1, GLO_M, GLO_M, GLO_M, GLO_M, GLO_K1, GLO_M, GLO_M,
         GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M, GLO_M,
         INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE};

static const uint8_t GALILEO_SAT_TYPE_SET[MAX_SAT_TYPE_NUM_GAL] =
        {GALILEO_2, GALILEO_2, GALILEO_2, GALILEO_2, GALILEO_2, INVALID_SAT_TYPE, GALILEO_2, GALILEO_2, //> prn 1-8
         GALILEO_2, INVALID_SAT_TYPE, GALILEO_1, GALILEO_1, GALILEO_2, GALILEO_2, GALILEO_2,
         INVALID_SAT_TYPE, //> prn 9-16
         INVALID_SAT_TYPE, GALILEO_2, GALILEO_1, GALILEO_1, GALILEO_2, GALILEO_2, INVALID_SAT_TYPE,
         GALILEO_2, //> prn 17 -24
         GALILEO_2, GALILEO_2, GALILEO_2, INVALID_SAT_TYPE, INVALID_SAT_TYPE, GALILEO_2, GALILEO_2, GALILEO_2,
         INVALID_SAT_TYPE,  //> prn 25-32
         GALILEO_2, INVALID_SAT_TYPE, INVALID_SAT_TYPE, GALILEO_2}; //> prn 33~

static const uint8_t BDS_SAT_TYPE_SET[MAX_SAT_TYPE_NUM_BDS] =
        {BDS_2G, BDS_2G, BDS_2G, BDS_2G, BDS_2G, BDS_2I, BDS_2I, BDS_2I,//> prn 1-8
         BDS_2I, BDS_2I, BDS_2M, BDS_2M, BDS_2I, BDS_2M, INVALID_SAT_TYPE, BDS_2I, //> prn 9-16
         INVALID_SAT_TYPE, INVALID_SAT_TYPE, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, //> prn 17-24
         BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3I, BDS_3M, //> prn 25-32
         BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3I, BDS_3I, BDS_3I, //> prn 33-40
         BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, BDS_3M, INVALID_SAT_TYPE, INVALID_SAT_TYPE, //> prn 41-48
         INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE,
         INVALID_SAT_TYPE, BDS_3I, //> prn 49-56
         BDS_3M, BDS_3M, BDS_3G, BDS_3G, BDS_3G, INVALID_SAT_TYPE, INVALID_SAT_TYPE, INVALID_SAT_TYPE, //> prn 57-64
         INVALID_SAT_TYPE //> prn 65
        };

/**
 * @brief input sat info
 * @param[in] this
 * @param[in] obs_time
 * @param[in] svid
 * @param[in] sys
 * @param[in] sat_coor
 * @param[in] sat_v
 */
void at_IntputSatInfo(SatAttitude_t *this, const GpsTime_t *obs_time, const int svid, const int sys,
                      const double sat_coor[3], const double sat_v[3])
{
  this->_sys_type = sys;
  this->_obs_time = *obs_time;
  this->_svid = svid;
  memcpy(this->_sat_coor, sat_coor, 3 * sizeof(double));
  memcpy(this->_sat_v, sat_v, 3 * sizeof(double));

  //transform the relative velocity to absolute velocity
  this->_sat_v[0] -= (ETHAV * sat_coor[1]);
  this->_sat_v[1] += (ETHAV * sat_coor[0]);
}

/**
 * @brief get satellte attitude
 * @param[in] this
 * @param[in] yaw
 * @param[out] x_point
 * @param[out] y_point
 * @param[out] z_point
 * @return bool
 */
bool at_GetAttitude(SatAttitude_t *this, const double *yaw, double x_point[3], double y_point[3], double z_point[3])
{
  at_Init(this);
  double xscf[3] = {0.0}, yscf[3] = {0.0}, zscf[3] = {0.0};

  if (yaw != NULL) {
    at_AttitudeCNES(this, *yaw, x_point, y_point, z_point);
    return true;
  }

  at_NominalYaw(this, xscf, yscf, zscf);
  for (int i = 0; i < 3; ++i)
  {
    x_point[i] = xscf[i];
    y_point[i] = yscf[i];
    z_point[i] = zscf[i];
  }
  double sat_dist = sqrt(gnss_Dot(this->_sat_coor, this->_sat_coor, 3));
  double sat_v_dist = sqrt(gnss_Dot(this->_sat_v, this->_sat_v, 3));

  // the actual satellite orbit angle rate(murate) (~0.00836 for GPS; ~0.00888 for GLONASS)
  this->_murate = sat_v_dist / sat_dist * RAD2DEG;  //unit: degree per second
  double v[3] = {this->_sat_v[0], this->_sat_v[1], this->_sat_v[2]};
  gnss_UnitVector(v, v, 3);
  this->_yaw_nominal = acos(gnss_Dot(xscf, v, 3)) * RAD2DEG;
  if (this->_beta0 > 0.0) {
    this->_yaw_nominal = -this->_yaw_nominal;
  }
  int iblk = at_SvBlock(this);
  if (iblk <= 0) {
    return false;
  }

  /* BDS2 */
  if (301 == iblk || 302 == iblk || 303 == iblk) {
    at_NonNominalYawBDS2(this, iblk, zscf, xscf);
    memcpy(z_point, zscf, 3 * sizeof(double));
    memcpy(x_point, xscf, 3 * sizeof(double));
    gnss_CrossMultiply(z_point, x_point, y_point);
    gnss_UnitVector(y_point, y_point, 3);
    return true;
  }
  else if (311 == iblk) {
    double sat_v_unit[3] = {0.0};
    gnss_UnitVector(this->_sat_v, sat_v_unit, 3);
    memcpy(z_point, zscf, 3 * sizeof(double));
    memcpy(x_point, sat_v_unit, 3 * sizeof(double));
    gnss_CrossMultiply(z_point, x_point, y_point);
    gnss_UnitVector(y_point, y_point, 3);
    return true;
  }

  this->_night_angle = 180 - (RE_WGS84 / sat_dist) * RAD2DEG;

  // night_threshold: unit degree
  // compute the noon beta angle limit(beta zero) for a noon turn from YRATEs
  double beta0 = 2.0; // TODO, beta0 not change
  at_YawRate(this, iblk, beta0);

  if (fabs(this->_yaw_rate) < 1.0e-12) {
    return false;
  }
  this->_noon_angle = atan(this->_murate / this->_yaw_rate) * RAD2DEG;
  if (fabs(beta0) > 1.0e-6) {
    beta0 = this->_noon_angle;
  }
  if (C_GNSS_GAL == this->_sys_type || C_GNSS_BDS3 == this->_sys_type) {
    this->_noon_angle = this->_night_angle - 180.0;
    this->_half_hour = 0.0;
  }
  this->_noon_cos = cos(this->_noon_angle * DEG2RAD);
  this->_night_cos = cos(this->_night_angle * DEG2RAD);

  at_ManeuverThreshold(this, this->_yaw_rate, beta0);

  if (4 == iblk || 5 == iblk || C_GNSS_GAL == this->_sys_type || C_GNSS_BDS3 == this->_sys_type) {
    this->_night_cos = cos((this->_noon_angle + 180.0) * DEG2RAD);
  }
  this->_bnight = this->_bnoon = false;
  this->_svb_cos = at_SvbCos(this->_sun_coor, this->_sat_coor);
  this->_bnight = this->_svb_cos < this->_night_cos ? true : false;
  this->_bnoon = this->_svb_cos > this->_noon_cos ? true : false;
  if ((C_GNSS_GAL == this->_sys_type || iblk > 311) && fabs(this->_beta0) >= beta0) {
    this->_bnight = this->_bnoon = false;
  }
  double phi = 0.0;
  at_NonNominalYaw(this, &phi, iblk);

  at_Xpoint(this, tm_GetMJD(&this->_obs_time), iblk, beta0, phi, xscf);

  memcpy(z_point, zscf, 3 * sizeof(double));
  memcpy(x_point, xscf, 3 * sizeof(double));
  gnss_CrossMultiply(z_point, x_point, y_point);
  gnss_UnitVector(y_point, y_point, 3);
  if (MANEUVER_NORMAL != this->_maneuver_type) {
    this->_is_decrease_weight = true;
  }
  if (C_GNSS_GPS == this->_sys_type) {
    if (fabs(fabs(this->_beta0) - 13.25) < 0.5) {
      this->_is_decrease_weight = true;
    }
  } else if (C_GNSS_GLO == this->_sys_type) {
    double beta0_abs = fabs(this->_beta0);
    if (fabs(beta0_abs - 14.2) < 0.5) {
      this->_is_decrease_weight = true;
    }
  }
  return true;
}

void at_Init(SatAttitude_t *this)
{
  for (int i = 0; i < 3; i++) {
    this->_orb_norm_unit[i] = 0.0;
  }
  this->_beta0 = 0.0;
  this->_bnoon = this->_bnight = false;
  this->_murate = this->_yaw_rate = this->_yaw_bias = this->_night_angle = this->_noon_angle = this->_noon_cos = this->_night_cos = this->_yaw_nominal = this->_svb_cos = this->_beta_ini = 0.0;
  this->_two_hour = 7200.0;
  this->_half_hour = 1800.0;
  this->_maneuver_type = MANEUVER_NORMAL;
  this->_is_decrease_weight = false;
}

bool at_NominalYaw(SatAttitude_t *this, double x_point[3], double y_point[3], double z_point[3])
{
  memset(x_point, 0, sizeof(double) * 3);
  memset(y_point, 0, sizeof(double) * 3);
  memset(z_point, 0, sizeof(double) * 3);
  double sat_coor_unit[3] = {0.0};

  //z
  gnss_UnitVector(this->_sat_coor, sat_coor_unit, 3);
  for (size_t i = 0; i < 3; i++) {
    z_point[i] = -sat_coor_unit[i];
  }

  //y
  double sat2sun_unit[3] = {0.0};
  for (size_t i = 0; i < 3; i++) {
    sat2sun_unit[i] = this->_sun_coor[i] - this->_sat_coor[i];
  }
  gnss_UnitVector(sat2sun_unit, sat2sun_unit, 3);
  double alpha = acos(gnss_Dot(z_point, sat2sun_unit, 3)) * RAD2DEG;
  double sat_v_unit[3] = {0.0};
  gnss_UnitVector(this->_sat_v, sat_v_unit, 3);

  if (alpha >= 1.0e-6 || alpha <= 180.0 - 1.0e-6) {
    gnss_CrossMultiply(z_point, sat2sun_unit, y_point);
  } else {
    gnss_CrossMultiply(z_point, sat_v_unit, y_point);
  }
  gnss_UnitVector(y_point, y_point, 3);

  //x
  gnss_CrossMultiply(y_point, z_point, x_point);
  gnss_UnitVector(x_point, x_point, 3);

  //calculation the sun elevation
  memset(this->_orb_norm_unit, 0, 3 * sizeof(double));
  gnss_CrossMultiply(sat_v_unit, z_point, this->_orb_norm_unit);
  gnss_UnitVector(this->_orb_norm_unit, this->_orb_norm_unit, 3);

  //double sun_coor    ???
  double sun_coor_unit[3] = {this->_sun_coor[0], this->_sun_coor[1],
                             this->_sun_coor[2]};
  gnss_UnitVector(sun_coor_unit, sun_coor_unit, 3);
  this->_beta0 = acos(gnss_Dot(this->_orb_norm_unit, sun_coor_unit, 3));
  this->_beta0 = 90 - this->_beta0 * RAD2DEG;//unit: degree
  return true;
}


bool
at_AttitudeCNES(SatAttitude_t *this, const double yaw, double x_point[3], double y_point[3], double z_point[3])
{
  memset(x_point, 0, sizeof(double) * 3);
  memset(y_point, 0, sizeof(double) * 3);
  memset(z_point, 0, sizeof(double) * 3);
  double sat_coor_unit[3] = {0.0};
  //z

  gnss_UnitVector(this->_sat_coor, sat_coor_unit, 3);
  for (size_t i = 0; i < 3; i++) {
    z_point[i] = -sat_coor_unit[i];
  }
  double X0[3] = {0.0};
  gnss_UnitVector(this->_sat_v, X0, 3);
  double Y0[3] = {0.0};
  gnss_CrossMultiply(this->_sat_coor, this->_sat_v, Y0);
  gnss_UnitVector(Y0, Y0, 3);
  double N[3] = {-Y0[0], -Y0[1], -Y0[2]};
  double sun_pos[3] = {0.0};
  gnss_UnitVector(this->_sun_coor, sun_pos, 3);

  double sin_beta = gnss_Dot(N, sun_pos, 3);
  // double beta_deg = asin(sin_beta) * RAD2DEG;
  double psi = yaw;
  if (fabs(psi) < 1.0e-10) {
    double u[3] = {0.0};
    gnss_CrossMultiply(sun_pos, N, u);
    gnss_UnitVector(u, u, 3);

    double tan_beta = sin_beta / sqrt(1.0 - sin_beta * sin_beta);
    double sinmu = gnss_Dot(u, sat_coor_unit, 3);
    psi = atan2(-tan_beta, sinmu);
    psi = psi < 0.0 ? (psi + PI * 2) : psi;
  }
  double cos_psi = cos(psi);
  double sin_psi = sin(psi);
  for (int i = 0; i < 3; i++) {
    x_point[i] = cos_psi * X0[i] + sin_psi * Y0[i];
    y_point[i] = cos_psi * Y0[i] - sin_psi * X0[i];
  }
  return true;
}

bool at_YawRate(SatAttitude_t *this, const int iblk, const double beta0)
{
  this->_yaw_rate = 0.0;
  this->_yaw_bias = 0.0;

  //check for BLOCK IIR and fix to nominal yaw rate
  switch (this->_sys_type) {
    case C_GNSS_GPS:
      //for BLOCK II/IIA
      this->_yaw_rate = ECLIP_YRATE_GPS[this->_svid-1];
      //for BLOCK IIR
      if (iblk >= 4) {
        this->_yaw_rate = 0.2;
      }
      if (iblk >= 6) {
        this->_yaw_rate = 0.11;
      }
      if (iblk <= 3) {
        this->_yaw_bias = 0.5;
      }
      if (iblk == 6) {
        this->_yaw_bias = -0.7;
      }
      if (iblk == 9) {
        this->_yaw_bias = 0.0;
      }
      break;
    case C_GNSS_GLO:
      this->_yaw_rate = 0.25;
      break;
    case C_GNSS_GAL:
      this->_yaw_rate = 0.203;
      this->_night_angle = 180.0 + 15.0;
      break;
    case C_GNSS_BDS3:
      this->_yaw_rate = 0.085;
      if (303 == iblk || 313 == iblk) {
        this->_yaw_rate = 0.158;
      }
      this->_night_angle = 180.0 + 15.0;
      break;
    default:
      break;
  }

  if (fabs(this->_yaw_bias) <= 0.01) {
    this->_yaw_bias = 0.0;
  }
  return true;
}

bool at_XpointBD2(SatAttitude_t *this, int iblk, double u, int *iyaw, double xscft[3])
{
  *iyaw = 0;
  double sat_v_unit[3] = {0.0};
  gnss_UnitVector(this->_sat_v, sat_v_unit, 3);
  if (301 == iblk) {
    *iyaw = 2;
    for (int i = 0; i < 3; i++) {
      xscft[i] = sat_v_unit[i];
    }
    return true;
  }
  this->_yaw_rate =
          this->_murate * tan(this->_beta0 * DEG2RAD) * cos(u) / (pow(sin(u), 2) + pow(tan(this->_beta0 * DEG2RAD), 2));
  if (fabs(this->_beta0) <= BDS2_BETA_MAX) {
    if (fabs(this->_yaw_nominal) > BDS2_YAW_MAX && fabs(this->_beta0) > (BDS2_BETA_MAX - 0.5)) {
// if beta is near to 4 deg and yaw is larger than thershold,
// make sure it has been switched to orbit-normal model or not
      if (false == this->_bds_LFirst) {
        *iyaw = this->_bds_iyaw_last;
        if (1 == *iyaw) {
          if (fabs(fabs(this->_yaw_nominal) - BDS2_YAW_MAX) < 15.0 && (this->_yaw_nominal * this->_yaw_rate) > 0.0) {
            *iyaw = 2;
          }
        }
      } else {
        *iyaw = 0;
        if (0 == *iyaw) {
          *iyaw = 2;
        }
      }
    } else {
      *iyaw = 2;
    }
  } else {
    *iyaw = 1;
    if (2 == this->_bds_iyaw_last && fabs(this->_yaw_nominal) > BDS2_YAW_MAX) {
      if (fabs(fabs(this->_yaw_nominal) - BDS2_YAW_MAX) < 15.0 && (this->_yaw_nominal * this->_yaw_rate > 0.0)) {
        *iyaw = 1;
      } else {
        *iyaw = 2;
      }
    }
  }
  this->_bds_LFirst = false;
  this->_bds_iyaw_last = *iyaw;
  if (2 == *iyaw) {
    for (int i = 0; i < 3; i++) {
      xscft[i] = sat_v_unit[i];
    }
  }
  return true;
}

double at_UAngle4Bd2(SatAttitude_t *this, const double zscf[3])
{
  double sun_coor_unit[3] = {0.0};
  gnss_UnitVector(this->_sun_coor, sun_coor_unit, 3);

  double sun_norm_unit[3] = {0.0};
  gnss_CrossMultiply(sun_coor_unit, this->_orb_norm_unit, sun_norm_unit);
  gnss_UnitVector(sun_norm_unit, sun_norm_unit, 3);

  double node_unit[3] = {0.0};
  gnss_CrossMultiply(sun_norm_unit, this->_orb_norm_unit, node_unit);
  gnss_UnitVector(node_unit, node_unit, 3);

  double neg_zscf[3] = {-zscf[0], -zscf[1], -zscf[2]};
  double fx = gnss_Dot(node_unit, neg_zscf, 3);

  double temp_unit[3] = {0.0};
  gnss_CrossMultiply(node_unit, neg_zscf, temp_unit);
  gnss_UnitVector(temp_unit, temp_unit, 3);

  double fz = gnss_Dot(temp_unit, this->_orb_norm_unit, 3);
  return atan2(fz, fx);
}

bool at_NonNominalYawBDS2(SatAttitude_t *this, int iblk, const double zscf[3], double xscf[3])
{
  double u = at_UAngle4Bd2(this, zscf);
  double xscft[3] = {xscf[0], xscf[1], xscf[2]};
  int iyaw = 0;
  at_XpointBD2(this, iblk, u, &iyaw, xscft);
  if (2 == iyaw) {
    for (int i = 0; i < 3; i++) {
      xscf[i] = xscft[i];
    }
  }
  return true;
}

bool at_OrbitalAngle(SatAttitude_t *this, int iblk, gnss_ManeuverType man_type, double* orbital_angle)
{
  *orbital_angle = 0.0;
  if (MANEUVER_NOON_TURN == man_type) {
    *orbital_angle = sqrt(pow(acos(this->_svb_cos) * RAD2DEG, 2) - pow(this->_beta0, 2));//miu from noon
    if (fabs(this->_yaw_nominal) > 90.0) {
      *orbital_angle = -*orbital_angle;
    }
  } else if (MANEUVER_NIGHT_TURN == man_type) {
    *orbital_angle = sqrt(pow(180.0 - acos(this->_svb_cos) * RAD2DEG, 2) - pow(this->_beta0, 2));//miu from midnight
    if (fabs(this->_yaw_nominal) < 90.0) {
      *orbital_angle = -*orbital_angle;
    }
  }
  return true;
}

bool at_TurnTimeTag(SatAttitude_t *this, int iblk, gnss_ManeuverType man_type, const double orbital_angle,
                    SatManeuverInfo_t* sat_man_info)
{
  //type of maneuver
  if (true == this->_bnoon) {
    sat_man_info->_man_type = MANEUVER_NOON_TURN;
  }
  if (true == this->_bnight) {
    sat_man_info->_man_type = MANEUVER_NIGHT_TURN;
  }
  if (MANEUVER_NOON_TURN == man_type) {
    if (C_GNSS_GLO == this->_sys_type || C_GNSS_GAL == this->_sys_type || iblk > 311) {
      sat_man_info->_endTime = sat_man_info->_startTime + (this->_noon_angle / this->_murate / 86400.0);
      sat_man_info->_startTime -= (this->_noon_angle / this->_murate / 86400.0);
    } else {
      double temp = (fabs(this->_beta0) * sqrt(this->_noon_angle / fabs(this->_beta0) - 1.0) / this->_murate) / 86400.0;
      sat_man_info->_endTime = sat_man_info->_startTime + temp;
      sat_man_info->_startTime -= temp;
    }
  } else if (MANEUVER_NIGHT_TURN == man_type) {
    double temp = (sqrt(pow(this->_night_angle - 180.0, 2) - pow(this->_beta0, 2)) / this->_murate) / 86400.0;
    sat_man_info->_endTime = sat_man_info->_startTime + temp;
    sat_man_info->_startTime -= temp;
  }
  return true;
}

bool at_NonNominalYaw(SatAttitude_t *this, double *phi, int iblk)
{
  double orbital_angle = 0.0;
  if (false == this->_bnight && false == this->_bnoon) {
    return true;
  }
  if (true == this->_bnight || true == this->_bnoon) {
    *phi = PI / 2.0;
    if (true == this->_bnight) {
      at_OrbitalAngle(this, iblk, MANEUVER_NIGHT_TURN, &orbital_angle);
      if (fabs(orbital_angle) > 1.0e-6) {
        *phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
      }
    }
    if (true == this->_bnoon) {
      at_OrbitalAngle(this, iblk, MANEUVER_NOON_TURN, &orbital_angle);
      if (fabs(orbital_angle) > 1.0e-6) {
        *phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(PI - orbital_angle * DEG2RAD)) * RAD2DEG;
      }
    }
    double mjd = (double) tm_GetMJD(&this->_obs_time);

    if (this->_sat_maneuver_num == 0) {
      SatManeuverInfo_t sat_man_info = {0};
      sat_man_info._startTime = mjd + (orbital_angle / this->_murate) / 86400.0;
      double yaw_end = atan(this->_murate / this->_yaw_rate) * RAD2DEG;
      if (((iblk > 3 && iblk <= 5) || true == this->_bnoon) && fabs(this->_beta0) < yaw_end) {
        at_TurnTimeTag(this, iblk, MANEUVER_NOON_TURN, orbital_angle, &sat_man_info);
      }
      if ((iblk <= 3 || iblk > 5 || C_GNSS_GAL == this->_sys_type || iblk > 311) && true == this->_bnight) {
        at_TurnTimeTag(this, iblk, MANEUVER_NIGHT_TURN, orbital_angle, &sat_man_info);
      }
      this->_sat_maneuver_set[this->_sat_maneuver_num++] = sat_man_info;

    }

    if (this->_sat_maneuver_num > 0 && ((true == this->_bnight && this->_svb_cos < this->_night_cos) ||
                                        (true == this->_bnoon && this->_svb_cos > this->_noon_cos))) {
      double dttag = fabs(mjd - this->_sat_maneuver_set[this->_sat_maneuver_num - 1]._startTime) * 86400.0;
      if ((C_GNSS_GPS == this->_sys_type || C_GNSS_GLO == this->_sys_type || C_GNSS_GAL == this->_sys_type ||
           iblk > 311)
          && fabs(this->_beta0) <= 0.07 && fabs(this->_beta_ini) < 1.0e-6) {
        this->_beta_ini = this->_beta0;
      }
      if (dttag > this->_two_hour) {
        SatManeuverInfo_t sat_man_info;
        sat_man_info._startTime = mjd + (orbital_angle / this->_murate) / 86400.0;
        if ((iblk > 3 && iblk <= 5) || true == this->_bnoon) {
          at_TurnTimeTag(this, iblk, MANEUVER_NOON_TURN, orbital_angle, &sat_man_info);
        }
        if ((iblk <= 3 || iblk > 5 || C_GNSS_GAL == this->_sys_type || iblk > 311) && true == this->_bnight) {
          at_TurnTimeTag(this, iblk, MANEUVER_NIGHT_TURN, orbital_angle, &sat_man_info);
        }
        if (this->_sat_maneuver_num == MAX_MANEUVER_SET_NUM) {
          memcpy(this->_sat_maneuver_set, this->_sat_maneuver_set + 1,
                 sizeof(SatManeuverInfo_t) * (MAX_MANEUVER_SET_NUM - 1));
          this->_sat_maneuver_set[MAX_MANEUVER_SET_NUM - 1] = sat_man_info;

        } else {
          this->_sat_maneuver_set[this->_sat_maneuver_num++] = sat_man_info;
        }
      }
    }
  }
  return true;
}


double at_SvbCos(double sun_coor[3], double sat_coor[3])
{
//get the unit vector of solar and satellite coordinate
  double sun_coor_unit[3] = {0.0}, sat_coor_unit[3] = {0.0};
  gnss_UnitVector(sun_coor, sun_coor_unit, 3);
  gnss_UnitVector(sat_coor, sat_coor_unit, 3);
//get the angle between sun_coor_unit and sat_coor_unit
  double svb_cos = gnss_Dot(sun_coor_unit, sat_coor_unit, 3);
  return svb_cos;
}


bool at_ManeuverThreshold(SatAttitude_t *this, const double yaw_rate, const double beta0)
{
  //get the threshold of night turn
  //GLONASS noon turn mode acording to DILSSNER 2010
  if ((C_GNSS_GLO == this->_sys_type) && fabs(this->_beta0) < this->_noon_angle) {
    double yaw_end = 75.0;
    //iteration for yaw_end of the GLONASS noon turn
    for (size_t i = 0; i < 3; i++) {
      double beta_tan = tan(this->_beta0 * DEG2RAD);
      double temp = this->_murate * yaw_end / yaw_rate * DEG2RAD;
      yaw_end = fabs(atan2(-beta_tan, sin(PI - temp)) -
                     atan2(-beta_tan, sin(PI + temp))) * RAD2DEG / 2.0;
    }
    //update anoon,cnoon for new GLONASS noon turn limits
    this->_noon_angle = (this->_murate * yaw_end / yaw_rate);
    this->_noon_cos = cos(this->_noon_angle * DEG2RAD);
  }
  return true;
}

bool at_Xpoint(SatAttitude_t *this, double obs_time, int iblk, const double beta0, double phi, double x_point[3])
{
  int nsize = this->_sat_maneuver_num;

  if (0 == nsize) {
    return true;
  }
  int index = -1;
  for (int i = 0; i < nsize; i++) {
    if (obs_time >= this->_sat_maneuver_set[i]._startTime &&
        obs_time <= this->_sat_maneuver_set[i]._endTime + this->_half_hour / 86400.0) {
      index = i;
    }
  }
  if (index < 0) {
    return true;
  }
  if (obs_time >= this->_sat_maneuver_set[index]._startTime &&
      obs_time <= (this->_sat_maneuver_set[index]._endTime + this->_half_hour / 86400.0)) {
    double arc_time = (this->_sat_maneuver_set[index]._endTime - this->_sat_maneuver_set[index]._startTime) * 86400.0;
    if (fabs(this->_beta0) <= 0.07 && (C_GNSS_GPS == this->_sys_type || C_GNSS_GLO == this->_sys_type) &&
        fabs(this->_beta_ini) > 1.0e-6) {
      this->_beta0 = this->_beta_ini;
    }
    double sat_coor_unit[3] = {0.0}, sat_v_unit[3] = {0.0};
    gnss_UnitVector(this->_sat_coor, sat_coor_unit, 3);
    gnss_UnitVector(this->_sat_v, sat_v_unit, 3);
    double orbital_angle = this->_murate * arc_time / 2.0;
    double yaw_end = atan2(-tan(this->_beta0 * DEG2RAD), sin(orbital_angle * DEG2RAD)) * RAD2DEG;
    yaw_end -= atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
    yaw_end /= arc_time;
    double delta_time = (obs_time - this->_sat_maneuver_set[index]._startTime) * 86400.0;
    if (this->_svb_cos < 0) {
      if (C_GNSS_GPS == this->_sys_type && (iblk <= 3 || iblk > 5)) {
        if (obs_time <= this->_sat_maneuver_set[index]._endTime) {
          if (iblk <= 3) {
            phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
            phi += at_Sign2(this->_yaw_rate, this->_yaw_bias) * delta_time;
          }
          if (iblk > 5) {
            phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
            phi += (yaw_end * delta_time);
          }
        } else {
          if (iblk <= 3) {
            phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
            phi += at_Sign2(this->_yaw_rate, this->_yaw_bias) * arc_time;
          }
          if (iblk > 5) {
            return true;
          }
          yaw_end = this->_yaw_nominal - phi;
          yaw_end = at_Tran180(yaw_end);
          phi += at_Sign2(this->_yaw_rate, yaw_end) * delta_time;
          double san_tx = this->_yaw_nominal - phi;
          san_tx = at_Tran180(san_tx);
          if (fabs(san_tx) > fabs(yaw_end))//stop! the nominal yaw(yangle) reached!
          {
            return true;
          }
          if (fabs(yaw_end) > 1.0e-6 && (san_tx / yaw_end) < 0.0)//stop! the nominal yaw(yangle) reached!
          {
            return true;
          }
          phi = at_Tran180(phi);
        }
      }
      if (C_GNSS_GPS == this->_sys_type && (iblk > 3 && iblk <= 5)) {
        phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
        phi += (at_Sign2(this->_yaw_rate, this->_beta0) * delta_time);
        if (this->_yaw_nominal / phi >= 1.0 || phi / this->_yaw_nominal < 0.0) {
          return true;
        }
      }
      if (C_GNSS_GLO == this->_sys_type) {
        if (obs_time > this->_sat_maneuver_set[index]._endTime) {
          return true;
        }
        yaw_end = this->_yaw_rate;
        phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(-orbital_angle * DEG2RAD)) * RAD2DEG;
        phi += (at_Sign2(yaw_end, this->_beta0) * delta_time);
        yaw_end = atan2(-tan(this->_beta0 * DEG2RAD), sin(orbital_angle * DEG2RAD)) * RAD2DEG;
        if (yaw_end / phi >= 1.0 || phi / yaw_end < 0.0) {
          phi = yaw_end;
        }
      }
      if (C_GNSS_GAL == this->_sys_type || iblk > 311) {
        double mid_time = (this->_sat_maneuver_set[index]._endTime + this->_sat_maneuver_set[index]._startTime) / 2.0;
        yaw_end = sin((obs_time - mid_time) * this->_murate * 86400.0 * DEG2RAD);
        double beta_e = 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta0) - sin(this->_beta0 * DEG2RAD))
                        + 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta0) + sin(this->_beta0 * DEG2RAD)) *
                          cos(PI * fabs(yaw_end * cos(this->_beta0 * DEG2RAD)) / sin(this->_noon_angle * DEG2RAD));
        if (fabs(this->_beta0) < 0.07) {
          if (fabs(this->_beta_ini) > 1.0e-6) {
            beta_e = 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta_ini) - sin(this->_beta0 * DEG2RAD))
                     + 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta_ini) + sin(this->_beta0 * DEG2RAD)) *
                       cos(PI * fabs(yaw_end * cos(this->_beta0 * DEG2RAD)) / sin(this->_noon_angle * DEG2RAD));
          }
        }
        phi = atan2(beta_e, yaw_end * cos(this->_beta0 * DEG2RAD)) * RAD2DEG;
      }
      this->_maneuver_type = MANEUVER_NIGHT_TURN;
    } else {
      phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(PI - orbital_angle * DEG2RAD)) * RAD2DEG -
              at_Sign2(this->_yaw_rate, this->_beta0) * delta_time;
      if ((C_GNSS_GPS == this->_sys_type) && ((this->_beta0 * at_Sign2(1.0, this->_yaw_bias)) <= fabs(this->_yaw_bias)) &&
          ((this->_beta0 * this->_yaw_bias) > 0.0)) {
        phi = atan2(-tan(this->_beta0 * DEG2RAD), sin(PI - orbital_angle * DEG2RAD)) * RAD2DEG +
                at_Sign2(this->_yaw_rate, this->_yaw_bias) * delta_time;
      }
      if (iblk > 3 && iblk <= 5) {
        if (phi / this->_yaw_nominal >= 1.0 || (phi / this->_yaw_nominal) < 0.0) {
          return true;
        }
      } else {
        if ((C_GNSS_GLO == this->_sys_type || C_GNSS_GAL == this->_sys_type || iblk > 311)
            && (obs_time > this->_sat_maneuver_set[index]._endTime)) {
          return true;
        }
        if ((C_GNSS_GPS == this->_sys_type) &&
            ((this->_beta0 * at_Sign2(1.0, this->_yaw_bias)) <= fabs(this->_yaw_bias)) &&
            ((this->_beta0 * this->_yaw_bias) > 0.0) &&
            ((((phi - at_Sign2(1.0, this->_yaw_bias) * 360.0) / this->_yaw_nominal) <= 1.0) ||
             (((phi - at_Sign2(1.0, this->_yaw_bias) * 360.0) / this->_yaw_nominal) < 0.0))) {
          return true;
        }
        if ((C_GNSS_GPS == this->_sys_type) &&
            ((this->_beta0 * at_Sign2(1.0, this->_yaw_bias)) > fabs(this->_yaw_bias) ||
             (this->_beta0 * this->_yaw_bias) <= 0.0)
            && (phi / this->_yaw_nominal >= 1.0 || phi / this->_yaw_nominal < 0.0)) {
          return true;
        }
      }
      if (C_GNSS_GAL == this->_sys_type || iblk > 311) {
        double mid_time = (this->_sat_maneuver_set[index]._endTime + this->_sat_maneuver_set[index]._startTime) / 2.0;
        yaw_end = sin(PI + (obs_time - mid_time) * this->_murate * 86400.0 * DEG2RAD);
        double beta_e = 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta0) - sin(this->_beta0 * DEG2RAD))
                        + 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta0) + sin(this->_beta0 * DEG2RAD)) *
                          cos(PI * fabs(yaw_end * cos(this->_beta0 * DEG2RAD)) / sin(this->_noon_angle * DEG2RAD));
        if (fabs(this->_beta0) < 0.07) {
          if (fabs(this->_beta_ini) > 1.0e-6) {
            beta_e = 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta_ini) - sin(this->_beta0 * DEG2RAD))
                     + 0.5 * (-at_Sign2(sin(beta0 * DEG2RAD), this->_beta_ini) + sin(this->_beta0 * DEG2RAD)) *
                       cos(PI * fabs(yaw_end * cos(this->_beta0 * DEG2RAD)) / sin(this->_noon_angle * DEG2RAD));
          }
        }
        phi = atan2(beta_e, yaw_end * cos(this->_beta0 * DEG2RAD)) * RAD2DEG;
      }
      this->_maneuver_type = MANEUVER_NOON_TURN;
    }
    double delta_alpha = phi - this->_yaw_nominal;
    delta_alpha *= (DEG2RAD);
    double yaw_cal = phi * DEG2RAD;
    double santx = (cos(delta_alpha) * (sat_v_unit[1] - sat_v_unit[2] * sat_coor_unit[1] / sat_coor_unit[2]) -
                    cos(yaw_cal) * (x_point[1] - x_point[2]
                                                 * sat_coor_unit[1] / sat_coor_unit[2])) /
                   (x_point[0] * sat_v_unit[1] - x_point[1] * sat_v_unit[0] +
                    ((x_point[1] * sat_v_unit[2] - x_point[2] * sat_v_unit[1]) * sat_coor_unit[0] +
                     (x_point[2] * sat_v_unit[0] - x_point[0] * sat_v_unit[2]) * sat_coor_unit[1]) / sat_coor_unit[2]);
    double santy = (cos(yaw_cal) - (sat_v_unit[0] - sat_v_unit[2] * sat_coor_unit[0] / sat_coor_unit[2]) * santx) /
                   (sat_v_unit[1] - sat_v_unit[2] * sat_coor_unit[1] / sat_coor_unit[2]);
    x_point[0] = santx;
    x_point[1] = santy;
    x_point[2] = (-sat_coor_unit[0] * santx - sat_coor_unit[1] * santy) / sat_coor_unit[2];
    gnss_UnitVector(x_point, x_point, 3);
  }
  return true;
}


// TODO: get satellite antenna type, atx file?
int at_SvBlock(SatAttitude_t *this)
{
  int iblk = 0;
  int idx = this->_svid - 1;

  if (C_GNSS_GPS == this->_sys_type) {
    if (idx >= MAX_SAT_TYPE_NUM_GPS) {
      return 0;
    }
    if (BLOCK_I == GPS_SAT_TYPE_SET[idx]) {
      iblk = 1;
    }
    else if (BLOCK_II == GPS_SAT_TYPE_SET[idx]) {
      iblk = 2;
    }
    else if (BLOCK_IIA == GPS_SAT_TYPE_SET[idx]) {
      iblk = 3;
    }
    else if (BLOCK_IIR == GPS_SAT_TYPE_SET[idx] || BLOCK_IIRA == GPS_SAT_TYPE_SET[idx]
             || BLOCK_IIRB == GPS_SAT_TYPE_SET[idx] || BLOCK_IIRM == GPS_SAT_TYPE_SET[idx]) {
      iblk = 4;
    }
    else if (BLOCK_IIF == GPS_SAT_TYPE_SET[idx]) {
      iblk = 6;
    }
    else if (BLOCK_IIIA == GPS_SAT_TYPE_SET[idx]) {
      iblk = 4;
    }
    else {
      return 0;
    }
  }
  else if (C_GNSS_GLO == this->_sys_type) {
    iblk = 6;
  }
  else if (C_GNSS_BDS3 == this->_sys_type) {
    if (idx >= MAX_SAT_TYPE_NUM_BDS) {
      return 0;
    }
    if (BDS_2G == BDS_SAT_TYPE_SET[idx]) {
      iblk = 301;
    }
    else if (BDS_2I == BDS_SAT_TYPE_SET[idx]) {
      iblk = 302;
    }
    else if (BDS_2M == BDS_SAT_TYPE_SET[idx]) {
      iblk = 303;
    }
    else if (BDS_3G == BDS_SAT_TYPE_SET[idx]) {
      iblk = 311;
    }
    else if (BDS_3I == BDS_SAT_TYPE_SET[idx]) {
      iblk = 312;
    }
    else if (BDS_3M == BDS_SAT_TYPE_SET[idx]) {
      iblk = 313;
    }
  }
  else if (C_GNSS_GAL == this->_sys_type) {
    if (idx >= MAX_SAT_TYPE_NUM_GAL) {
      return 0;
    }
    if (GALILEO_GIOVEA == GALILEO_SAT_TYPE_SET[idx] || GALILEO_GIOVEB == GALILEO_SAT_TYPE_SET[idx]) {
      iblk = 201;
    }
    else if (GALILEO_IOV == GALILEO_SAT_TYPE_SET[idx] || GALILEO_1 == GALILEO_SAT_TYPE_SET[idx]) {
      iblk = 202;
    }
    else if (GALILEO_FOC == GALILEO_SAT_TYPE_SET[idx] || GALILEO_2 == GALILEO_SAT_TYPE_SET[idx]) {
      iblk = 203;
    }
    else if (INVALID_SAT_TYPE == GALILEO_SAT_TYPE_SET[idx]) {
      iblk = 203;
    }

  }
  return iblk;
}

double at_Sign2(const double x, const double y)
{
  return y >= 0.0 ? fabs(x) : -fabs(x);
}

double at_Tran180(const double x)
{
  double result = 0.0;
  result = fmod(x, 360.0);
  //result: -180 ~ +180
  if (fabs(result) > 180.0) {
    result -= (360.0 * result / fabs(result));
  }
  return result;
}
