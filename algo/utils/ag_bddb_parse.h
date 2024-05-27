/**@file        ag_bddb_parse.h
 * @brief       asensing bddb data parse header file
 * @version     V1.0
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/05/29  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __AG_BDDB_PARSE_H__
#define __AG_BDDB_PARSE_H__

#include <stdint.h>
#include "cmn_def.h"
#include "gnss_type.h"
#include "fusion_api.h"
#include "loc_core_api.h"

BEGIN_DECL


//#define OLD_BDDB10   /*BDDB10 without SNR*/

#ifdef OLD_BDDB10
#define LEN_BD_DB_10 (70u)
#else
#define LEN_BD_DB_10 (74u)
#endif
#define LEN_BD_DB_A8 (720u+4u)
#define LEN_BD_DB_98 (4u)
#define LEN_BD_DB_11 (45u)
#define LEN_BD_DB_A9 (4u)
#define LEN_BD_DB_65 (4u)
#define LEN_BD_DB_08 (4u)
#define LEN_BD_DB_0A (34u)
#define LEN_BD_DB_0B (58u)
#define LEN_BD_DB_1B (69u)
#define LEN_BD_DB_12 (46u)
#define LEN_BD_DB_20 (34u)
#define LEN_BD_DB_E3 (11u)
#define LEN_BD_DB_E5 (5u)
#define LEN_BD_DB_E6 (11u)
#define LEN_BD_DB_36 (9u)


#define AG_IMU_BDDB_DECODED_BUFFER_SIZE (1024)

typedef struct {
  uint32_t nbyte;
  uint32_t length;
  uint8_t  prev_ch;
  uint8_t  u_type;
  uint8_t  buf[AG_IMU_BDDB_DECODED_BUFFER_SIZE];
} ag_imu_bddb_decoder_t;


uint8_t ag_imu_bddb_decode(ag_imu_bddb_decoder_t* pz_decoder, uint8_t ch, uint8_t* p_data, uint32_t* length);

void ag_imu_bddb_parse_0x0A(uint8_t* p_data, uint32_t q_length, ag_imu_data_t* pz_ImuData);

uint8_t ag_imu_bddb_parse_0x0B(uint8_t* p_data, uint32_t q_length, INSResults_t* pz_InsData);

void ag_imu_bddb_parse_0x10(uint8_t* p_data, uint32_t q_length, GnssFixType* pz_Gnss);

void ag_imu_bddb_parse_0x20(uint8_t* p_data, uint32_t q_length, loc_api_wheel_t* pz_Wheel);

uint32_t ag_imu_bddb_encode_0x0A(AsensingIMU_t* pz_ImuData, int8_t* p_output);

uint32_t ag_imu_bddb_encode_0x10(AsensingGNSSPara_t* pz_GnssFix, int8_t* p_output);

uint32_t ag_imu_bddb_encode_0x20(AsensingWhpulse_t* pz_WheelData, int8_t* p_output);

uint32_t ag_imu_bddb_encode_0x0B(INSResults_t* pz_InsPositionFix, int8_t* p_output);

uint32_t ag_imu_bddb_encode_0x0D(INSResults_t* pz_InsPositionFix, int8_t* p_output);
END_DECL

#endif
