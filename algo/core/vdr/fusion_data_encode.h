/**@file        fusion_data_encode.h
 * @brief		fusion data encode header file
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

#ifndef _FUSION_DATA_ENCODE_H_
#define _FUSION_DATA_ENCODE_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "fusion_api.h"
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a): (b))
#endif // !MIN

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a): (b))
#endif // !MIN

#define LSB_16(x)    ((x)&0x000000FFu)
#define MSB_16(x)    (((x)>>8u)&0x000000FFu)
#define LSB_32(x)    ((x)&0x000000FFu)
#define MID1_32(x)   (((x)>>8u)&0x000000FFu)
#define MID2_32(x)   (((x)>>16u)&0x000000FFu)
#define MSB_32(x)    (((x)>>24u)&0x000000FFu)

#define X2INT16(x)    MIN(MAX(((int32_t)(x)),-32768),32767)
#define X2UINT16(x)   MIN((uint32_t)(x),65535u)
#define X2INT32(x)     ((int32_t)(x))
#define LONG2INT32(x)  X2INT32(((x)*1e7+0.5))
#define LAT2INT32(x)   X2INT32(((x)*1e7+0.5))
#define H2INT32(x)     X2INT32(((float)(x))*1e3f)
#define X2UINT32(x)		((uint32_t)(x))

#define LITTLE_ENDIAN 0
#define BIG_ENDIAN 1

void data_ReadImuData(const uint8_t* input, AsensingIMU_t* output);
void data_ReadGnssData(const uint8_t* input, AsensingGNSSPara_t* output);
void data_ReadWheelData(const uint8_t* input, AsensingWhpulse_t* output);
void data_ReadPPSData(const uint8_t* input, AsensingPPS_t* output);
void data_ReadLeverData(const uint8_t* input, float* lever);
void data_ReadTwoRearData(const uint8_t* input, float* lever);
void data_ReadMisAngleData(const uint8_t* input, float* att);
void data_ReadHistoryData(const uint8_t* input, HistoryInsResults_t* output);
void data_ReadPsrMeasData(const uint8_t* input, AsensingGNSSPsrMeas_t* output);
void data_ReadCpMeasData(const uint8_t* input, AsensingGNSSCpMeas_t* output);
int8_t fusion_decode_SelfCheck(uint8_t* str, int32_t len);
uint32_t data_encode_BDDBAA(AsensingIMU_t* pz_input, uint8_t* pu_output);
uint32_t data_encode_BDDBCC(AsensingWhpulse_t* pz_input, uint8_t* pu_output);
uint32_t data_encode_BDDBDD(AsensingPPS_t* pz_input, uint8_t* pu_output);
uint32_t data_encode_BDDB0A(AsensingIMU_t* pz_ImuData, uint8_t* p_output);
uint32_t data_encode_BDDB0B(INSResults_t* pz_InsPositionFix, uint8_t* p_output);
uint32_t data_encode_BDDB0D(INSIntegrity_t* pz_integrity, uint8_t* p_output);
uint32_t data_encode_BDDB0E(AsensingGNSSPara_t* pz_GnssFix, int8_t* p_output);
uint32_t data_encode_BDDB10(AsensingGNSSPara_t* pz_GnssFix, uint8_t* p_output);
uint32_t data_encode_BDDB20(AsensingWhpulse_t* pz_WheelData, uint8_t* p_output);
#ifdef __cplusplus
}
#endif

#endif // !_FUSION_DATA_ENCODE_H_
