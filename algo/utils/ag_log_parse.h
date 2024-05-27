/**@file        pb_ag_parse.h
 * @brief       pe playback AG format data parse
 * @version     V1.0
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/24  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __PB_AG_PARSE_H__
#define __PB_AG_PARSE_H__

#include <stdint.h>
#include "mw_log.h"
#include "mw_ipctask.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_DECODER_SIZE (64 * 1024)
#define LOG_PRECHECK_LEN (15) // 1 + 2 (HEADER) + 2(TYPE) + 4(LENGTH) + 4(CRC)

typedef struct
{
  uint8_t  u_input_buf[LOG_DECODER_SIZE];
  uint8_t  prev_ch;
  uint32_t nbyte;
  uint32_t length;
  uint16_t w_type;
  uint8_t  u_decoded_buf[LOG_DECODER_SIZE];
  uint32_t u_decoded_length;
} log_decoder_t;

typedef void (*log_parsed_data_handler)(int8_t decode_type, uint8_t* p_data, uint32_t q_length);

/**
 * @brief Decode log buffer
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    Length Byte data                       | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|       playload data                       | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |

IPC : |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
Text: |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|             Log Text String               | CHECKSUM|
Pack: |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      Pkg ID         |      package data   | CHECKSUM|
Cmprs:|LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|             Compressed data               | CHECKSUM|

 * 
 * @param[in] ipc
 * @return: None
 */
int8_t ag_log_decode(log_decoder_t* decoder, uint8_t ch, uint8_t* p_data, uint32_t* length);

/**
* @brief Parse the input log data
* @param[in] pz_log_decoder    : The pointer to log decoder
* @param[in] fp_hander         : Parsed log text output hander
* @param[in] p_data            : Input data
* @param[in] q_length          : Input data length
* @param[in] b_compress_enable : Indicate to support parse compressed mode or not
* @return None
*/
void loc_core_log_parse(log_decoder_t* pz_log_decoder, log_parsed_data_handler p_hander,
  uint8_t* p_data, uint32_t q_length, uint8_t b_compress_enable);

#ifdef __cplusplus
}
#endif

#endif