/**@file        fusion_mech.h
 * @brief		fusion mechanization header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_MECH_H__
#define __FUSION_MECH_H__

#include <stdint.h>

#define MECH_DEFAULT_DT 0.005f
#define SYNC_QUEUE_SIZE 20

typedef struct
{
  double d_mech_time;
  double d_midpos[3];
  double d_prev_imu[10];
  float  f_mechan_t;
  float  f_delta_t;
  float  f_midvel[3];
  float  f_coning[3];
  float  f_sculling[3];
} MechanParas_t;

typedef struct
{
  double d_time;
  double d_lla[3];
  double d_qne[4];	
  double d_rm;
  double d_rn;
  float  f_vel[3];
  float  f_deta_vel[3];
  float  f_qbn[4];
  float  f_fb[3];
  float  f_fn[3];
  float  f_gb[3];
  float  f_wien[3];
  float  f_wnen[3];
  float  f_wibb[3];	
} MechanNode_t;

typedef struct
{
  MechanNode_t z_nodes[SYNC_QUEUE_SIZE];
  MechanNode_t z_nmech;
  MechanNode_t z_mech_bds;
  MechanNode_t z_mech_veh;
  uint8_t u_count;
  uint8_t u_read_index;
  uint8_t u_write_index;
} SyncQueue_t;

void mechan_initial(void);
void mechan_set_time(double d_time);
double mechan_get_time(void);
int8_t mechan_get_node(MechanNode_t* pz_node);

#endif // !__FUSION_MECH_H__
