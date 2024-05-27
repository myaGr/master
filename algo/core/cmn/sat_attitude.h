/**@file        sat_attitude.h
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

#ifndef __SAT_ATTITUDE_H__
#define __SAT_ATTITUDE_H__

#include "gnss_type.h"


#define MAX_MANEUVER_SET_NUM 100
#define MAX_SAT_TYPE_NUM_GPS  40
#define MAX_SAT_TYPE_NUM_GLO  40
#define MAX_SAT_TYPE_NUM_BDS  65
#define MAX_SAT_TYPE_NUM_GAL  40 

typedef enum
{
  BLOCK_I = 0,
  BLOCK_II,
  BLOCK_IIA,
  BLOCK_IIIA,
  BLOCK_IIR,
  BLOCK_IIRA,
  BLOCK_IIRB,
  BLOCK_IIRM,
  BLOCK_IIF,
  GLO,
  GLO_M,
  GLO_K1,
  BDS_2G,
  BDS_2I,
  BDS_2M,
  BDS_3G,
  BDS_3I,
  BDS_3M,
  GALILEO_1,
  GALILEO_2,
  GALILEO_GIOVEA,
  GALILEO_GIOVEB,
  GALILEO_IOV,
  GALILEO_FOC,
  QZSS_SAT,
  SBAS_SAT,
  INVALID_SAT_TYPE
} sat_type;

typedef enum {
  MANEUVER_NORMAL,
  MANEUVER_NOON_TURN,
  MANEUVER_NIGHT_TURN,
  MANEUVER_SHADOW_RECOVERY,
  MANEUVER_MANEUVER_NUM
} gnss_ManeuverType;

typedef struct
{
  double _startTime;
  double _endTime;
  gnss_ManeuverType _man_type;
} SatManeuverInfo_t;

typedef struct
{
  GpsTime_t _obs_time;
  gnss_ConstellationType _sys_type;
  int _svid;
  double _sat_coor[3];
  double _sat_v[3];
  double _sun_coor[3];
  double _beta0;
  int _sat_maneuver_num;
  SatManeuverInfo_t _sat_maneuver_set[MAX_MANEUVER_SET_NUM];
  bool _bnoon;
  bool _bnight;
  double _murate;
  double _yaw_rate;
  double _yaw_bias;
  double _night_angle;   /* night beta angle (degree) */
  double _noon_angle;    /* noon beta angle (degree) */
  double _noon_cos;
  double _night_cos;
  double _yaw_nominal;
  double _svb_cos;
  double _two_hour;
  double _half_hour;
  double _beta_ini;
  gnss_ManeuverType _maneuver_type;
  double _orb_norm_unit[3];
  bool _bds_LFirst;
  int _bds_iyaw_last;
  bool _is_decrease_weight;
} SatAttitude_t;

/**
 * @brief input sat info
 * @param[in] this
 * @param[in] obs_time
 * @param[in] svid
 * @param[in] sys
 * @param[in] sat_coor
 * @param[in] sat_v
 */
void at_IntputSatInfo(SatAttitude_t* this, const GpsTime_t* obs_time, const int svid, const int sys,
  const double sat_coor[3], const double sat_v[3]);
/**
 * @brief get satellte attitude
 * @param[in] this
 * @param[in] yaw
 * @param[out] x_point
 * @param[out] y_point
 * @param[out] z_point
 * @return bool
 */
bool at_GetAttitude(SatAttitude_t* this, const double* yaw, double x_point[3], double y_point[3], double z_point[3]);

bool at_YawRate(SatAttitude_t* this, const int iblk, const double beta0);

void at_Init(SatAttitude_t* this);

bool at_AttitudeCNES(SatAttitude_t* this, const double yaw, double x_point[3], double y_point[3], double z_point[3]);

bool at_NominalYaw(SatAttitude_t* this, double x_point[3], double y_point[3], double z_point[3]);

bool at_ManeuverThreshold(SatAttitude_t* this, const double yaw_rate, const double beta0);

bool at_Xpoint(SatAttitude_t* this, double obs_time, int iblk, const double beta0, double phi, double x_point[3]);

bool at_XpointBD2(SatAttitude_t* this, int iblk, double u, int* iyaw, double xscft[3]);

double at_UAngle4Bd2(SatAttitude_t* this, const double zscf[3]);

bool at_NonNominalYawBDS2(SatAttitude_t* this, int iblk, const double zscf[3], double xscf[3]);

bool at_NonNominalYaw(SatAttitude_t* this, double* phi, int iblk);

bool at_OrbitalAngle(SatAttitude_t* this, int iblk, gnss_ManeuverType man_type, double* orbital_angle);

bool at_TurnTimeTag(SatAttitude_t* this, int iblk, gnss_ManeuverType man_type, const double orbital_angle,
  SatManeuverInfo_t* sat_man_info);

int at_SvBlock(SatAttitude_t* this);

double at_SvbCos(double sun_coor[3], double sat_coor[3]);

double at_Sign2(const double x, const double y);

double at_Tran180(const double x);

#endif // !__SAT_ATTITUDE_H__
