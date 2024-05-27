/**@file        fusion_data_encode.c
 * @brief		fusion data encode file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/01/5   <td>0.1      <td>shaobing    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_math.h"
#include "fusion_data_encode.h"
#include "fusion_err_model.h"

#define KWS_INIT 0.00863 

int8_t fusion_decode_SelfCheck(uint8_t* str, int32_t len)
{
  uint8_t u_checkA = 0;
  uint8_t u_checkB = 0;
  uint32_t i;

  for (i = 0; i < (uint32_t)(len - 2); i++)
  {
    u_checkA ^= str[i];
    u_checkB ^= u_checkA;
  }
  if (u_checkA == str[len - 2] && u_checkB == str[len - 1])
  {
    return 1;
  }
  return 0;
}

static float data_Byte2Float(const uint8_t* input, uint8_t type)
{
  uint32_t buf = 0;
  float ret;
  if (type == 1)
  {
    buf = (((uint32_t)input[0]) << 24) + (((uint32_t)input[1]) << 16) + (((uint32_t)input[2]) << 8) + (((uint32_t)input[3]) << 0);
  }
  else
  {
    buf = (((uint32_t)input[0]) << 0) + (((uint32_t)input[1]) << 8) + (((uint32_t)input[2]) << 16) + (((uint32_t)input[3]) << 24);
  }
  ret = *((float*)(&buf));
  return ret;
}

static uint64_t data_Byte2UInt64(const uint8_t* input, uint8_t type)
{
  uint64_t ret = 0;
  if (type == 1)
  {
    ret = (((uint64_t)input[0]) << 56) + (((uint64_t)input[1]) << 48) + (((uint64_t)input[2]) << 40) + (((uint64_t)input[3]) << 32) +
      (((uint64_t)input[4]) << 24) + (((uint64_t)input[5]) << 16) + (((uint64_t)input[6]) << 8) + (((uint64_t)input[7]) << 0);
  }
  else
  {
    ret = (((uint64_t)input[0]) << 0) + (((uint64_t)input[1]) << 8) + (((uint64_t)input[2]) << 16) + (((uint64_t)input[3]) << 24) +
      (((uint64_t)input[4]) << 32) + (((uint64_t)input[5]) << 40) + (((uint64_t)input[6]) << 48) + (((uint64_t)input[7]) << 56);
  }
  return ret;
}

static int16_t data_Byte2Int16(const uint8_t* input, uint8_t type)
{
  uint16_t buf = 0;
  int16_t ret;

  if (type == 1)
  {
    buf = (uint16_t)((((uint32_t)input[0]) << 8) + (((uint32_t)input[1]) << 0));
  }
  else
  {
    buf = (uint16_t)((((uint32_t)input[0]) << 0) + (((uint32_t)input[1]) << 8));
  }
  ret = *((int16_t*)(&buf));
  return ret;
}

static uint16_t data_Byte2UInt16(const uint8_t* input, uint8_t type)
{
  uint16_t ret = 0;

  if (type == 1)
  {
    ret = (uint16_t)((((uint32_t)input[0]) << 8) + (((uint32_t)input[1]) << 0));
  }
  else
  {
    ret = (uint16_t)((((uint32_t)input[0]) << 0) + (((uint32_t)input[1]) << 8));
  }
  return ret;
}

static int32_t data_Byte2Int32(const uint8_t* input, uint8_t type)
{
  uint32_t buf;
  int32_t ret;

  if (type == 1)
  {
    buf = (((uint32_t)input[0]) << 24) + (((uint32_t)input[1]) << 16) + (((uint32_t)input[2]) << 8) + (((uint32_t)input[3]) << 0);
  }
  else
  {
    buf = (((uint32_t)input[0]) << 0) + (((uint32_t)input[1]) << 8) + (((uint32_t)input[2]) << 16) + (((uint32_t)input[3]) << 24);
  }
  ret = *((int32_t*)(&buf));
  return ret;
}

static uint32_t data_Byte2UInt32(const uint8_t* input, uint8_t type)
{
  uint32_t ret;

  if (type == 1)
  {
    ret = (((uint32_t)input[0]) << 24) + (((uint32_t)input[1]) << 16) + (((uint32_t)input[2]) << 8) + (((uint32_t)input[3]) << 0);
  }
  else
  {
    ret = (((uint32_t)input[0]) << 0) + (((uint32_t)input[1]) << 8) + (((uint32_t)input[2]) << 16) + (((uint32_t)input[3]) << 24);
  }
  return ret;
}

static void encode_double(double data, uint8_t* buf, uint32_t* index)
{
  uint32_t buff_uint32 = 0;
  int32_t buff_int32 = 0;

  buff_int32 = (int32_t)(data);
  buff_uint32 = *((uint32_t*)(&buff_int32));
  buf[(*index)++] = (uint8_t)(buff_uint32);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 8);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 16);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 24);

  buff_int32 = (int32_t)((data - buff_int32) * 1.0e8);
  buff_uint32 = *((uint32_t*)(&buff_int32));
  buf[(*index)++] = (uint8_t)(buff_uint32);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 8);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 16);
  buf[(*index)++] = (uint8_t)(buff_uint32 >> 24);
  return;
}

void encode_double2int32(double value, uint8_t* write_buff, uint32_t* index)
{
  uint32_t buff_uint32 = 0;
  int32_t buff_int32 = 0;
  buff_int32 = (int32_t)(value);
  buff_uint32 = *((uint32_t*)(&buff_int32));
  write_buff[(*index)++] = (uint8_t)(buff_uint32);
  write_buff[(*index)++] = (uint8_t)(buff_uint32 >> 8);
  write_buff[(*index)++] = (uint8_t)(buff_uint32 >> 16);
  write_buff[(*index)++] = (uint8_t)(buff_uint32 >> 24);
  return;
}

double decode_double(const uint8_t* input, uint8_t edn)
{
  double res = 0.0;
  int32_t res_int = 0;
  int32_t res_sec = 0;

  res_int = data_Byte2Int32(input, edn);
  res_sec = data_Byte2Int32(input + 4, edn);
  res = (double)(res_int + res_sec / 1.0e8);
  return res;
}

void data_ReadImuData(const uint8_t* input, AsensingIMU_t* output)
{
  output->t_timestamp = data_Byte2UInt64(&input[3], LITTLE_ENDIAN);
  output->f_gyro[0] = data_Byte2Float(&input[11], LITTLE_ENDIAN);
  output->f_gyro[1] = data_Byte2Float(&input[15], LITTLE_ENDIAN);
  output->f_gyro[2] = data_Byte2Float(&input[19], LITTLE_ENDIAN);

  output->f_accl[0] = data_Byte2Float(&input[23], LITTLE_ENDIAN);
  output->f_accl[1] = data_Byte2Float(&input[27], LITTLE_ENDIAN);
  output->f_accl[2] = data_Byte2Float(&input[31], LITTLE_ENDIAN);

  output->f_temp = (float)(((float)data_Byte2Int32(&input[35], LITTLE_ENDIAN)) / 1e7L);
  output->u_selfck = input[39];
}

void data_ReadGnssData(const uint8_t* input, AsensingGNSSPara_t* output)
{
  output->t_timestamp = data_Byte2UInt64(&input[3], LITTLE_ENDIAN);
  output->t_timeStampOfUtc = data_Byte2UInt64(&input[11], LITTLE_ENDIAN);
  output->d_lat = ((double)data_Byte2Int32(&input[19], LITTLE_ENDIAN)) / 1e7L;
  output->d_lon = ((double)data_Byte2Int32(&input[23], LITTLE_ENDIAN)) / 1e7L;
  output->f_alt = (float)(((double)data_Byte2Int32(&input[27], LITTLE_ENDIAN)) / 1e3f);
  output->f_speed = ((float)data_Byte2Int16(&input[31], LITTLE_ENDIAN)) / 1e2f;
  output->f_heading = ((float)data_Byte2Int16(&input[33], LITTLE_ENDIAN)) / 1e2f;
  output->f_pitch = ((float)data_Byte2Int16(&input[35], LITTLE_ENDIAN)) / 1e2f;
  output->f_hdop = ((float)data_Byte2UInt16(&input[37], LITTLE_ENDIAN)) / 1e3f;
  output->f_hor_accu = ((float)data_Byte2UInt16(&input[39], LITTLE_ENDIAN)) / 1e3f;
  output->f_vn = (float)(((float)data_Byte2Int32(&input[41], LITTLE_ENDIAN)) / 1e7L);
  output->f_ve = (float)(((float)data_Byte2Int32(&input[45], LITTLE_ENDIAN)) / 1e7L);
  output->f_vd = (float)(((float)data_Byte2Int32(&input[49], LITTLE_ENDIAN)) / 1e7L);
  output->f_latstd = ((float)data_Byte2UInt16(&input[53], LITTLE_ENDIAN)) / 1e3f;
  output->f_lonstd = ((float)data_Byte2UInt16(&input[55], LITTLE_ENDIAN)) / 1e3f;
  output->f_altstd = ((float)data_Byte2UInt16(&input[57], LITTLE_ENDIAN)) / 1e3f;
  output->f_vnstd = ((float)data_Byte2UInt16(&input[59], LITTLE_ENDIAN)) / 1e3f;
  output->f_vestd = ((float)data_Byte2UInt16(&input[61], LITTLE_ENDIAN)) / 1e3f;
  output->f_vdstd = ((float)data_Byte2UInt16(&input[63], LITTLE_ENDIAN)) / 1e3f;
  output->d_tow = (double)(data_Byte2UInt32(&input[65], LITTLE_ENDIAN)) / 1e3f;
  output->f_avgsnr = ((float)data_Byte2Int16(&input[69], LITTLE_ENDIAN)) / 1e2f;
  output->w_week = data_Byte2UInt16(&input[71], LITTLE_ENDIAN);
  output->u_postype = input[73];
  output->u_satused = input[74];
  output->e_posflag = (GnssPosFlag_Enum)data_Byte2UInt32(&input[75], LITTLE_ENDIAN);
  output->u_msg_type = input[79];
}

void data_ReadWheelData(const uint8_t* input, AsensingWhpulse_t* output)
{
  output->t_timestamp = data_Byte2UInt64(&input[3], LITTLE_ENDIAN);
  output->q_fl_whpulse = (uint32_t)data_Byte2UInt16(&input[11], LITTLE_ENDIAN);
  output->q_fr_whpulse = (uint32_t)data_Byte2UInt16(&input[13], LITTLE_ENDIAN);
  output->q_rl_whpulse = (uint32_t)data_Byte2UInt16(&input[15], LITTLE_ENDIAN);
  output->q_rr_whpulse = (uint32_t)data_Byte2UInt16(&input[17], LITTLE_ENDIAN);
  output->f_angle_front = ((float)data_Byte2UInt16(&input[19], LITTLE_ENDIAN)) * 0.1f;
  output->f_angle_rear = ((float)data_Byte2UInt16(&input[21], LITTLE_ENDIAN)) * 0.1f;
  output->f_odometer = ((float)data_Byte2UInt16(&input[23], LITTLE_ENDIAN)) * 0.1f;
  output->e_gear = (GearShift_Enum)input[25];
  output->u_selfck = input[26];
  output->t_shifter_ms = data_Byte2UInt64(&input[27], LITTLE_ENDIAN);
  output->t_angle_ms = data_Byte2UInt64(&input[35], LITTLE_ENDIAN);
}

void data_ReadPPSData(const uint8_t* input, AsensingPPS_t* output)
{
  output->t_timestamp = data_Byte2UInt64(&input[3], LITTLE_ENDIAN);
  output->w_valid = data_Byte2UInt16(&input[11], LITTLE_ENDIAN);
}

void data_ReadLeverData(const uint8_t* input, float* lever)
{
  lever[0] = ((float)data_Byte2Int16(&input[3], LITTLE_ENDIAN)) / 1e2f;
  lever[1] = ((float)data_Byte2Int16(&input[5], LITTLE_ENDIAN)) / 1e2f;
  lever[2] = ((float)data_Byte2Int16(&input[7], LITTLE_ENDIAN)) / 1e2f;
}

void data_ReadTwoRearData(const uint8_t* input, float* lever)
{
  lever[0] = ((float)data_Byte2Int16(&input[3], LITTLE_ENDIAN)) / 1e2f;
}

void data_ReadMisAngleData(const uint8_t* input, float* att)
{
  att[0] = ((float)data_Byte2Int16(&input[3], LITTLE_ENDIAN)) / 1e2f;
  att[1] = ((float)data_Byte2Int16(&input[5], LITTLE_ENDIAN)) / 1e2f;
  att[2] = ((float)data_Byte2Int16(&input[7], LITTLE_ENDIAN)) / 1e2f;
}

void data_ReadHistoryData(const uint8_t* input, HistoryInsResults_t* output)
{
  output->u_validType = input[3];
  output->d_lat = ((double)data_Byte2Int32(&input[4], LITTLE_ENDIAN)) / 1e7L;
  output->d_lon = ((double)data_Byte2Int32(&input[8], LITTLE_ENDIAN)) / 1e7L;
  output->f_alt = (float)(((double)data_Byte2Int32(&input[12], LITTLE_ENDIAN)) / 1e3f);
  output->f_vn = ((float)data_Byte2Int16(&input[16], LITTLE_ENDIAN)) / 1e2f;
  output->f_ve = ((float)data_Byte2Int16(&input[18], LITTLE_ENDIAN)) / 1e2f;
  output->f_vd = ((float)data_Byte2Int16(&input[20], LITTLE_ENDIAN)) / 1e2f;
  output->f_roll = ((float)data_Byte2Int16(&input[22], LITTLE_ENDIAN)) / 1e2f;
  output->f_pitch = ((float)data_Byte2Int16(&input[24], LITTLE_ENDIAN)) / 1e2f;
  output->f_yaw = ((float)data_Byte2Int16(&input[26], LITTLE_ENDIAN)) / 1e2f;
  output->f_mis_roll = ((float)data_Byte2Int16(&input[28], LITTLE_ENDIAN)) / 1e2f;
  output->f_mis_pitch = ((float)data_Byte2Int16(&input[30], LITTLE_ENDIAN)) / 1e2f;
  output->f_mis_yaw = ((float)data_Byte2Int16(&input[32], LITTLE_ENDIAN)) / 1e2f;
  output->u_v_axis_mode = input[34];
  output->u_h_axis_mode = input[35];
}

void data_ReadPsrMeasData(const uint8_t* input, AsensingGNSSPsrMeas_t* output)
{
  uint32_t i, offset;
  output->u_count = input[3];
  output->d_cur_tow = ((double)data_Byte2Int32(&input[4], LITTLE_ENDIAN)) / 1e3L;
  output->d_blh[0] = decode_double(&input[8], LITTLE_ENDIAN);
  output->d_blh[1] = decode_double(&input[16], LITTLE_ENDIAN);
  output->d_blh[2] = decode_double(&input[24], LITTLE_ENDIAN);
  output->e_posflag = (uint8_t)input[32];
  for (i = 0; i < output->u_count; i++)
  {
    offset = 74 * i;
    output->z_raw_data[i].d_pr = decode_double(&input[33 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_dr = decode_double(&input[41 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_pos[0] = decode_double(&input[49 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_pos[1] = decode_double(&input[57 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_pos[2] = decode_double(&input[65 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_vel[0] = decode_double(&input[73 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_vel[1] = decode_double(&input[81 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].d_sat_vel[2] = decode_double(&input[89 + offset], LITTLE_ENDIAN);
    output->z_raw_data[i].f_prnoise = ((float)data_Byte2Int16(&input[97 + offset], LITTLE_ENDIAN)) / 1e2f;
    output->z_raw_data[i].f_drnoise = ((float)data_Byte2Int16(&input[99 + offset], LITTLE_ENDIAN)) / 1e2f;
    output->z_raw_data[i].f_ele = ((float)data_Byte2Int16(&input[101 + offset], LITTLE_ENDIAN)) / 1e2f;
    output->z_raw_data[i].u_prn = input[103 + offset];
    output->z_raw_data[i].u_sys = input[104 + offset];
    output->z_raw_data[i].u_snr = input[105 + offset];
    output->z_raw_data[i].u_valid = input[106 + offset];
  }
}

void data_ReadCpMeasData(const uint8_t* input, AsensingGNSSCpMeas_t* output)
{
  uint32_t i, offset;
  output->u_count = input[3];
  output->d_cur_tow = ((double)data_Byte2Int32(&input[4], LITTLE_ENDIAN)) / 1e3L;
  output->d_blh[0] = decode_double(&input[8], LITTLE_ENDIAN);
  output->d_blh[1] = decode_double(&input[16], LITTLE_ENDIAN);
  output->d_blh[2] = decode_double(&input[24], LITTLE_ENDIAN);
  output->e_posflag = (uint8_t)input[32];
  output->e_diff_mask = (uint8_t)input[33];
  for (i = 0; i < output->u_count; i++)
  {
    offset = 62 * i;
    output->z_raw_cp_data[i].f_tdcp_obs = (float)(((double)data_Byte2Int32(&input[34 + offset], LITTLE_ENDIAN)) / 1e4f);
    output->z_raw_cp_data[i].f_tdcp_res = (float)(((double)data_Byte2Int32(&input[38 + offset], LITTLE_ENDIAN)) / 1e4f);
    output->z_raw_cp_data[i].d_cur_sat_pos[0] = decode_double(&input[42 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].d_cur_sat_pos[1] = decode_double(&input[50 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].d_cur_sat_pos[2] = decode_double(&input[58 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].d_pre_sat_pos[0] = decode_double(&input[66 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].d_pre_sat_pos[1] = decode_double(&input[74 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].d_pre_sat_pos[2] = decode_double(&input[82 + offset], LITTLE_ENDIAN);
    output->z_raw_cp_data[i].f_ele = ((float)data_Byte2Int16(&input[90 + offset], LITTLE_ENDIAN)) / 1e2f;
    output->z_raw_cp_data[i].u_prn = input[92 + offset];
    output->z_raw_cp_data[i].u_sys = input[93 + offset];
    output->z_raw_cp_data[i].u_snr = input[94 + offset];
    output->z_raw_cp_data[i].u_valid = input[95 + offset];
  }
}

uint32_t data_encode_BDDBAA(AsensingIMU_t* pz_input, uint8_t* pu_output)
{
  uint32_t index = 0;
  float* pf_buf;
  float f_buf;
  uint32_t q_bufU32;
  uint64_t t_bufU64;

  pu_output[index++] = 0xBD;
  pu_output[index++] = 0xDB;
  pu_output[index++] = 0xAA;

  t_bufU64 = pz_input->t_timestamp;
  pu_output[index++] = (uint8_t)t_bufU64;
  pu_output[index++] = (uint8_t)(t_bufU64 >> 8);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 16);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 24);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 32);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 40);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 48);
  pu_output[index++] = (uint8_t)(t_bufU64 >> 56);

  f_buf = pz_input->f_gyro[0];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  f_buf = pz_input->f_gyro[1];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  f_buf = pz_input->f_gyro[2];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  f_buf = pz_input->f_accl[0];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  f_buf = pz_input->f_accl[1];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  f_buf = pz_input->f_accl[2];
  pf_buf = &f_buf;
  q_bufU32 = *((uint32_t*)pf_buf);
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  int32_t tt = (int32_t)(pz_input->f_temp * 1e7L);
  q_bufU32 = (uint32_t)tt;
  pu_output[index++] = (uint8_t)q_bufU32;
  pu_output[index++] = (uint8_t)(q_bufU32 >> 8);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 16);
  pu_output[index++] = (uint8_t)(q_bufU32 >> 24);

  pu_output[index++] = pz_input->u_selfck;

  uint8_t u_checkA = 0;
  uint8_t u_checkB = 0;
  uint32_t i;
  for (i = 0; i < index; i++)
  {
    u_checkA ^= pu_output[i];
    u_checkB ^= u_checkA;
  }
  pu_output[index++] = u_checkA;
  pu_output[index++] = u_checkB;

  return index;
}

uint32_t data_encode_BDDBCC(AsensingWhpulse_t* pz_input, uint8_t* pu_output)
{
  uint32_t index = 0;

  pu_output[index++] = 0xBD;
  pu_output[index++] = 0xDB;
  pu_output[index++] = 0xCC;

  pu_output[index++] = (uint8_t)(pz_input->t_timestamp);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 8);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 16);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 24);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 32);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 40);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 48);
  pu_output[index++] = (uint8_t)((pz_input->t_timestamp) >> 56);

  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_fl_whpulse);
  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_fl_whpulse >> 8);

  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_fr_whpulse);
  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_fr_whpulse >> 8);

  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_rl_whpulse);
  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_rl_whpulse >> 8);

  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_rr_whpulse);
  pu_output[index++] = (uint8_t)((uint32_t)pz_input->q_rr_whpulse >> 8);

  pu_output[index++] = (uint8_t)(pz_input->f_angle_front * 10.0f);
  pu_output[index++] = (uint8_t)((uint32_t)(pz_input->f_angle_front * 10.0f) >> 8);

  pu_output[index++] = (uint8_t)(pz_input->f_angle_rear * 10.0f);
  pu_output[index++] = (uint8_t)((uint32_t)(pz_input->f_angle_rear * 10.0f) >> 8);

  pu_output[index++] = (uint8_t)(pz_input->f_odometer * 10.0f);
  pu_output[index++] = (uint8_t)((uint32_t)(pz_input->f_odometer * 10.0f) >> 8);

  pu_output[index++] = (uint8_t)pz_input->e_gear;
  pu_output[index++] = (uint8_t)pz_input->u_selfck;

  pu_output[index++] = (uint8_t)(pz_input->t_shifter_ms);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 8);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 16);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 24);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 32);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 40);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 48);
  pu_output[index++] = (uint8_t)((pz_input->t_shifter_ms) >> 56);

  pu_output[index++] = (uint8_t)(pz_input->t_angle_ms);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 8);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 16);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 24);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 32);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 40);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 48);
  pu_output[index++] = (uint8_t)((pz_input->t_angle_ms) >> 56);

  uint8_t u_checkA = 0;
  uint8_t u_checkB = 0;
  uint32_t i;
  for (i = 0; i < index; i++)
  {
    u_checkA ^= pu_output[i];
    u_checkB ^= u_checkA;
  }
  pu_output[index++] = u_checkA;
  pu_output[index++] = u_checkB;

  return index;
}

uint32_t data_encode_BDDBDD(AsensingPPS_t* pz_input, uint8_t* pu_output)
{
  uint32_t index = 0;
  uint64_t t_buf;

  pu_output[index++] = 0xBD;
  pu_output[index++] = 0xDB;
  pu_output[index++] = 0xDD;

  t_buf = (pz_input->t_timestamp);
  pu_output[index++] = (uint8_t)t_buf;
  pu_output[index++] = (uint8_t)(t_buf >> 8);
  pu_output[index++] = (uint8_t)(t_buf >> 16);
  pu_output[index++] = (uint8_t)(t_buf >> 24);
  pu_output[index++] = (uint8_t)(t_buf >> 32);
  pu_output[index++] = (uint8_t)(t_buf >> 40);
  pu_output[index++] = (uint8_t)(t_buf >> 48);
  pu_output[index++] = (uint8_t)(t_buf >> 56);

  pu_output[index++] = (uint8_t)(pz_input->w_valid);
  pu_output[index++] = (uint8_t)(pz_input->w_valid >> 8);

  uint8_t u_checkA = 0;
  uint8_t u_checkB = 0;
  uint8_t i;
  for (i = 0; i < index; i++)
  {
    u_checkA ^= pu_output[i];
    u_checkB ^= u_checkA;
  }
  pu_output[index++] = u_checkA;
  pu_output[index++] = u_checkB;

  return index;
}

static float AgMsg_Pdata2TrueVal(float in)
{
  return (logf(in) * 50.f);
}

uint32_t data_encode_BDDB0A(AsensingIMU_t* pz_ImuData, uint8_t* p_output)
{
  uint8_t i = 0;
  uint32_t index = 0;

  uint8_t Check_A = 0;
  uint8_t Check_B = 0;

  float* pf;
  float buf;
  uint32_t buf_u32;
  uint16_t buf_u16;
  int16_t buf_s16;

  p_output[index++] = 0xBD;
  p_output[index++] = 0xDB;
  p_output[index++] = 0x0A;

  buf = (float)(pz_ImuData->f_gyro[0] * RAD2DEG);
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf = (float)(pz_ImuData->f_gyro[1] * RAD2DEG);
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf = (float)(pz_ImuData->f_gyro[2] * RAD2DEG);
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf = pz_ImuData->f_accl[0] / Gravity_GuangZhou;
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf = pz_ImuData->f_accl[1] / Gravity_GuangZhou;
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf = pz_ImuData->f_accl[2] / Gravity_GuangZhou;
  pf = &buf;
  buf_u32 = *((uint32_t*)pf);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);

  buf_s16 = (int16_t)(pz_ImuData->f_temp * 32768.0f / 200.0f);
  buf_u16 = *((uint16_t*)(&buf_s16));
  p_output[index++] = (uint8_t)(buf_u16 >> 0u);
  p_output[index++] = (uint8_t)(buf_u16 >> 8u);

  buf_u32 = (uint32_t)(pz_ImuData->t_timestamp * 4.0);
  p_output[index++] = (uint8_t)(buf_u32 >> 0u);
  p_output[index++] = (uint8_t)(buf_u32 >> 8u);
  p_output[index++] = (uint8_t)(buf_u32 >> 16u);
  p_output[index++] = (uint8_t)(buf_u32 >> 24u);


  for (i = 0u; i < index; i++)
  {
    Check_A ^= p_output[i];
    Check_B ^= Check_A;
  }
  p_output[index++] = Check_A;
  p_output[index++] = Check_B;

  return index;
}

uint32_t data_encode_BDDB0B(INSResults_t* pz_InsPositionFix, uint8_t* p_output)
{
  static uint8_t Index = 0u;
  uint8_t i = 0u, j;
  uint8_t checkbyte = 0u;
  uint16_t buf_u16;
  int16_t buf_i16;
  int32_t buf_i32;
  uint32_t buf_u32;
  int16_t temp_P[3] = {0};

  p_output[i++] = 0xBDu;
  p_output[i++] = 0xDBu;
  p_output[i++] = 0x0Bu;

  buf_i16 = (int16_t)(pz_InsPositionFix->f_roll / (360.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(pz_InsPositionFix->f_pitch / (360.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(pz_InsPositionFix->f_heading / (360.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i32 = (int32_t)(pz_InsPositionFix->d_latitude * RAD2DEG * 1e7);
  buf_u32 = *((uint32_t*)(&buf_i32));
  p_output[i++] = (uint8_t)(buf_u32);
  p_output[i++] = (uint8_t)(buf_u32 >> 8u);
  p_output[i++] = (uint8_t)(buf_u32 >> 16u);
  p_output[i++] = (uint8_t)(buf_u32 >> 24u);

  buf_i32 = (int32_t)(pz_InsPositionFix->d_longitude * RAD2DEG * 1e7);
  buf_u32 = *((uint32_t*)(&buf_i32));
  p_output[i++] = (uint8_t)(buf_u32);
  p_output[i++] = (uint8_t)(buf_u32 >> 8u);
  p_output[i++] = (uint8_t)(buf_u32 >> 16u);
  p_output[i++] = (uint8_t)(buf_u32 >> 24u);

  buf_i32 = (int32_t)(pz_InsPositionFix->f_altitude * 1e3);
  buf_u32 = *((uint32_t*)(&buf_i32));
  p_output[i++] = (uint8_t)(buf_u32);
  p_output[i++] = (uint8_t)(buf_u32 >> 8u);
  p_output[i++] = (uint8_t)(buf_u32 >> 16u);
  p_output[i++] = (uint8_t)(buf_u32 >> 24u);

  buf_i16 = (int16_t)(pz_InsPositionFix->f_vn / (100.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(pz_InsPositionFix->f_ve / (100.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  buf_i16 = (int16_t)(pz_InsPositionFix->f_vd / (100.0f / 32768.0f));
  buf_u16 = *((uint16_t*)(&buf_i16));
  p_output[i++] = (uint8_t)(buf_u16);
  p_output[i++] = (uint8_t)(buf_u16 >> 8u);

  p_output[i++] = (uint8_t)(pz_InsPositionFix->e_drposflag >= 6 ? 31 : 1);

  p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 0u);
  p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 8u);
  p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 16u);
  p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 24u);

  p_output[i++] = (uint8_t)1;

  p_output[i++] = (uint8_t)0;

  switch (Index)
  {
  case 0u:		 /* Ppos */
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_lat_std * pz_InsPositionFix->f_lat_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_lon_std * pz_InsPositionFix->f_lon_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_alt_std * pz_InsPositionFix->f_alt_std);
    break;
  case 1u:		 /* Pv */
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_vn_std * pz_InsPositionFix->f_vn_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_ve_std * pz_InsPositionFix->f_ve_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_vd_std * pz_InsPositionFix->f_vd_std);
    break;
  case 2u:		 /* Patt */
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_roll_std * pz_InsPositionFix->f_roll_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_pitch_std * pz_InsPositionFix->f_pitch_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_yaw_std * pz_InsPositionFix->f_yaw_std);
    break;
  case 3u:		 /* Pba */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 4u:		 /* Pbg */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 5u:		 /* Xpos */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 6u:		 /* Xv */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 7u:		 /* Xatt */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 8u:		 /* Xba */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 9u:		 /* Xbg */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 10u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_mis_roll / 360.0f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_mis_pitch / 360.0f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_mis_yaw / 360.0f * 32768.0f);
    break;
  case 11:		/* PCtb */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 12:		 /* XCtb */
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 13u:
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 14u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[0] * 1000.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[1] * 1000.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[2] * 1000.0f);
    break;
  case 17u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[0] * 1000.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[1] * 1000.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[2] * 1000.0f);
    break;
  case 20u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_gyrobias_x * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_gyrobias_y * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_gyrobias_z * 32768.0f);
    break;
  case 21u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_accbias_x / Gravity_GuangZhou / 0.1f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_accbias_y / Gravity_GuangZhou / 0.1f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_accbias_z / Gravity_GuangZhou / 0.1f * 32768.0f);
    break;
  case 22u:
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 23u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 24:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_misdualant_roll / 360.0f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_misdualant_pitch / 360.0f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_misdualant_yaw / 360.0f * 32768.0f);
    break;
  case 27u:
    temp_P[0] = (int16_t)(KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[2]) * 1e5f);
    temp_P[1] = (int16_t)(KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[3]) * 1e5f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_rear2rear * 1e3f);
    break;
  case 30u:
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 32u:		/* gps */
    temp_P[0] = (int16_t)0; /* GET_NOVATEL_FLAG_POS(gpsdata->flags) */
    temp_P[1] = (int16_t)0; /* gpsdata->SatNumPos */
    temp_P[2] = (int16_t)0; /* GET_NOVATEL_FLAG_HEADING(gpsdata->flags) */
    break;
  case 33u:  /* 571 old */
    temp_P[0] = X2INT16(0 * 1e2);
    temp_P[1] = X2INT16(0 * 1e2);
    temp_P[2] = X2INT16(0 * 1e2);
    break;
  case 36u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 44u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 45u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 46u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 51u:
    temp_P[0] = (int16_t)(0 * 1e3f);
    temp_P[1] = (int16_t)(0 * 1e3f);
    temp_P[2] = (int16_t)(0 * 1e3f);
    break;
  case 52u:
    temp_P[0] = X2INT16(0 * 1e10f);
    temp_P[1] = X2INT16(0 * 1e10f);
    temp_P[2] = X2INT16(0 * 1e3f);
    break;
  case 54u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 55u:
    temp_P[0] = X2INT16(0 * 1e3f);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  default:
    temp_P[0] = 0u;
    temp_P[1] = 0u;
    temp_P[2] = 0u;
    break;
  }

  p_output[i++] = (uint8_t)(temp_P[0]);
  p_output[i++] = (uint8_t)(temp_P[0] >> 8u);
  p_output[i++] = (uint8_t)(temp_P[1]);
  p_output[i++] = (uint8_t)(temp_P[1] >> 8u);
  p_output[i++] = (uint8_t)(temp_P[2]);
  p_output[i++] = (uint8_t)(temp_P[2] >> 8u);

  buf_u32 = (uint32_t)(pz_InsPositionFix->q_tow * 4.0);
  p_output[i++] = (uint8_t)(buf_u32);
  p_output[i++] = (uint8_t)(buf_u32 >> 8u);
  p_output[i++] = (uint8_t)(buf_u32 >> 16u);
  p_output[i++] = (uint8_t)(buf_u32 >> 24u);

  p_output[i++] = Index++;
  if (Index > 55u)
  {
    Index = 0u;
  }

  for (j = 0u; j < i; j++)
  {
    checkbyte ^= p_output[j];
  }
  p_output[i++] = (uint8_t)checkbyte;
  return i;
}

uint32_t data_encode_BDDB0D(INSIntegrity_t* pz_integrity, uint8_t* p_output)
{
  uint8_t i = 0u, j = 0u;
  uint8_t Check_A = 0u;
  uint32_t uintTemp;

  j = 0u;
  p_output[j++] = 0xBDu;
  p_output[j++] = 0xDBu;
  p_output[j++] = 0x0Du;

  uintTemp = X2UINT16(pz_integrity->f_pos_longitudinal_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pos_lateral_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pos_hpl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pos_vpl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pos_north_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pos_east_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_longitudinal_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_lateral_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_hpl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_vpl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_north_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_vel_east_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_roll_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_pitch_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_integrity->f_yaw_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  p_output[j++] = (uint8_t)(pz_integrity->e_drposflag);
  
  p_output[j++] = (uint8_t)(((uint32_t)pz_integrity->t_timestamp));
  p_output[j++] = (uint8_t)(((uint32_t)pz_integrity->t_timestamp) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_integrity->t_timestamp) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_integrity->t_timestamp) >> 24u);

  for (i = 0u; i < j; i++)
  {
    Check_A ^= p_output[i];
  }
  p_output[j++] = Check_A;
  return j;
}

uint32_t data_encode_BDDB0E(AsensingGNSSPara_t* pz_GnssFix, int8_t* p_output)
{
  uint8_t i = 0u, j = 0u;
  uint8_t Check_A = 0u;
  uint32_t uintTemp;

  j = 0u;
  p_output[j++] = 0xBDu;
  p_output[j++] = 0xDBu;
  p_output[j++] = 0x0Eu;

  uintTemp = X2UINT16(pz_GnssFix->f_lat_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_lon_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_alt_pl * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  p_output[j++] = (uint8_t)(((uint32_t)pz_GnssFix->d_tow));
  p_output[j++] = (uint8_t)(((uint32_t)pz_GnssFix->d_tow) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_GnssFix->d_tow) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_GnssFix->d_tow) >> 24u);

  for (i = 0u; i < j; i++)
  {
    Check_A ^= p_output[i];
  }
  p_output[j++] = Check_A;
  return j;
}

uint32_t data_encode_BDDB20(AsensingWhpulse_t* pz_WheelData, uint8_t* p_output)
{
  uint32_t j, i;
  uint8_t Check_A = 0u;
  uint8_t Check_B = 0u;

  j = 0u;
  p_output[j++] = 0xBDu;
  p_output[j++] = 0xDBu;
  p_output[j++] = 0x20u;

  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fl_whpulse);
  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fl_whpulse >> 8u);

  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fr_whpulse);
  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fr_whpulse >> 8u);

  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

  p_output[j++] = 0u;

  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rl_whpulse);
  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rl_whpulse >> 8u);

  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rr_whpulse);
  p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rr_whpulse >> 8u);

  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

  p_output[j++] = 0u;

  p_output[j++] = (uint8_t)pz_WheelData->e_gear;

  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

  p_output[j++] = (uint8_t)(pz_WheelData->f_angle_front * 10.0f);
  p_output[j++] = (uint8_t)((uint32_t)(pz_WheelData->f_angle_front * 10.0f) >> 8u);

  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
  p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);
  p_output[j++] = 0u;

  for (i = 0u; i < j; i++)
  {
    Check_A ^= p_output[i];
    Check_B ^= Check_A;
  }
  p_output[j++] = Check_A;
  p_output[j++] = Check_B;
  return j;
}

uint32_t data_encode_BDDB10(AsensingGNSSPara_t* pz_GnssFix, uint8_t* p_output)
{
  uint8_t i = 0u, j = 0u;
  uint8_t checkbyte_a = 0u, checkbyte_b = 0u;
  uint32_t uintTemp;
  int32_t intTemp;

  j = 0u;
  p_output[j++] = 0xBDu;
  p_output[j++] = 0xDBu;
  p_output[j++] = 0x10u;

  intTemp = LONG2INT32(pz_GnssFix->d_lon);
  p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_lonstd * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  intTemp = LAT2INT32(pz_GnssFix->d_lat);
  p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_latstd * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  intTemp = H2INT32(pz_GnssFix->f_alt);
  p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_altstd * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  uintTemp = X2UINT16(pz_GnssFix->f_hor_accu * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

  p_output[j++] = (uint8_t)(0 + 0.5f);

  p_output[j++] = (uint8_t)(0);

  p_output[j++] = pz_GnssFix->e_posflag;
  p_output[j++] = pz_GnssFix->e_posflag;
  p_output[j++] = pz_GnssFix->e_posflag;
  p_output[j++] = pz_GnssFix->e_posflag;

  intTemp = X2INT16(pz_GnssFix->f_speed * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_heading * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_vd * 1e2f * (-1.0));
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(0 * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(0 * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_heading_deg * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_heading_std * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_pitch_deg * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  intTemp = X2INT16(pz_GnssFix->f_pitch_std * 1e3f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

  uintTemp = (((pz_GnssFix->w_week % 2000u) & 0x3ffu) << 6u) + ((pz_GnssFix->UtcYear % 2000u) & 0x3fu);
  p_output[j++] = (uint8_t)LSB_16(uintTemp);
  p_output[j++] = (uint8_t)MSB_16(uintTemp);

  p_output[j++] = (uint8_t)(pz_GnssFix->UtcMon);
  p_output[j++] = (uint8_t)(pz_GnssFix->UtcDay);
  p_output[j++] = (uint8_t)(pz_GnssFix->UtcHour);
  p_output[j++] = (uint8_t)(pz_GnssFix->UtcMin);

  uintTemp = X2UINT16(pz_GnssFix->UtcSec * 1000u);
  p_output[j++] = (uint8_t)LSB_16(uintTemp);
  p_output[j++] = (uint8_t)MSB_16(uintTemp);

  uintTemp = X2UINT32(pz_GnssFix->d_tow);
  p_output[j++] = (uint8_t)LSB_32(uintTemp);
  p_output[j++] = (uint8_t)MID1_32(uintTemp);
  p_output[j++] = (uint8_t)MID2_32(uintTemp);
  p_output[j++] = (uint8_t)MSB_32(uintTemp);

  uintTemp = X2UINT32(pz_GnssFix->d_tow);
  p_output[j++] = (uint8_t)LSB_32(uintTemp);
  p_output[j++] = (uint8_t)MID1_32(uintTemp);
  p_output[j++] = (uint8_t)MID2_32(uintTemp);
  p_output[j++] = (uint8_t)MSB_32(uintTemp);

  uintTemp = X2UINT32(pz_GnssFix->d_tow); /* itow_heading */
  p_output[j++] = (uint8_t)LSB_32(uintTemp);
  p_output[j++] = (uint8_t)MID1_32(uintTemp);
  p_output[j++] = (uint8_t)MID2_32(uintTemp);
  p_output[j++] = (uint8_t)MSB_32(uintTemp);

  p_output[j++] = (uint8_t)LSB_32(7);

  p_output[j++] = pz_GnssFix->u_satused;

  for (i = 0u; i < j; i++)
  {
    checkbyte_a ^= p_output[i];
    checkbyte_b ^= checkbyte_a;
  }
  p_output[j++] = checkbyte_a;
  p_output[j++] = checkbyte_b;

  intTemp = X2INT16(pz_GnssFix->f_avgsnr * 1e2f);
  p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
  p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);
  checkbyte_a = 0u;

  for (i = 0u; i < j; i++)
  {
    checkbyte_a ^= p_output[i];
  }
  p_output[j++] = checkbyte_a;
  return j;
}