/**@file        fusion_axis_detect.h
 * @brief		fusion axis detect header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_AXIS_DETECT_H__
#define __FUSION_AXIS_DETECT_H__

#include <stdint.h>

#define FLEX_AXIS_STATIC_BUF_LEN    30U
#define FLEX_AXIS_FIT_WINDOW_TIME   30U
#define FLEX_AXIS_BUFFER_HZ         1
#define FLEX_AXIS_FIT_WINDOW_LENGTH (FLEX_AXIS_FIT_WINDOW_TIME)
#define FLEX_AXIS_ANGLE_BUF_LEN     5

typedef struct
{
  float bA[3];
  float kA[3];
  float rA[3];
  float k_m[3];
} LSFitPara_t;

typedef struct
{
  uint8_t  cnt;
  double timeTag;
  float  gyro[3];
  float  acc[3];
  float  odometer;
} IMUMsg_t;

typedef struct
{
  double timeTag;
  float acc;
  float vel;
  float flexAxisAngleBuf[FLEX_AXIS_ANGLE_BUF_LEN];
  uint8_t flexAxisAngleBufCnt;
  uint8_t qualityFlag;
} GNSSMsg_t;

typedef struct
{
  float accData[3][FLEX_AXIS_FIT_WINDOW_LENGTH];
  float odo[FLEX_AXIS_FIT_WINDOW_LENGTH];
  uint32_t cnt;
  uint8_t fullFlag;
} IMUMsgBuf_t;

typedef struct
{
  double timeTag[FLEX_AXIS_FIT_WINDOW_LENGTH]; /* for debug purpose */
  float data[FLEX_AXIS_FIT_WINDOW_LENGTH];
  uint32_t cnt;
  uint8_t fullFlag;
} GNSSMsgBuf_t;

typedef struct
{
  float buf[3][FLEX_AXIS_STATIC_BUF_LEN];
  uint8_t cnt;
  float staticValue[3];
  uint8_t preZuptFlag;
} AcclHPStatic_t;

typedef struct
{
  float f_acc_xyz[3];
  float f_acc_tmp[3];
  uint32_t q_count;
  uint8_t u_axis_mode;
  uint8_t u_last_axis_mode;
  uint8_t u_vert_detected;
} VerticalAxis_t;

typedef struct
{
  IMUMsg_t z_imumsg;
  IMUMsg_t z_imu_cpy;
  GNSSMsg_t z_gnssmsg;
  IMUMsgBuf_t z_imubuf;
  GNSSMsgBuf_t z_gnssbuf;
  AcclHPStatic_t z_acc_static;
  VerticalAxis_t z_vert_axis;
  LSFitPara_t z_ls_para;
  float  f_acclp[3];
  float  f_gyrolp[3];
  float f_axiscfg[9];
  float f_miseuler[3];
  float f_preodo_val;
  uint32_t q_fliter_cnt;
  uint8_t u_iready_flag;
  uint8_t u_gready_flag;
  uint8_t u_axis_flag;
  uint8_t u_misalign_flag;
  uint8_t u_dataloss_flag;
  uint8_t u_is_detected;
} FlxAxisDectct_t;

typedef struct
{
  int32_t AxisFlag;
  int32_t AxisHeading;
  float AccAvgValue[3];
} FlxAxisResult_t;

void flx_axis_init(void);
void flx_detect_triaxis(double* pd_meas);
void flx_convert_imu_axis_by_history(HistoryInsResults_t* pz_history, double* pd_mea);
void flx_init_misAngle_by_history(HistoryInsResults_t* pz_history);
int32_t flx_set_axis_mode(int32_t mode);
FlxAxisDectct_t* fusion_get_flx_axis(void);
void flx_config_misalign(NavConfig_t* pz_navconfig, FlxAxisDectct_t* pz_flx);

#endif // !__FUSION_AXIS_DETECT_H__

