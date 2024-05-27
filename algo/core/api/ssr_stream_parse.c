/**@file        ssr_stream_parse.c
 * @brief       SSR Stream parse
 * @version     V1.0
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/24  <td>0.1      <td>zhanglei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "mw_alloc.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "ssr_stream_parse.h"
#include "loc_core_report.h"

static uint8_t   BitU1(uint8_t* p) { uint8_t   val; memcpy(&val, p, 1); return val; }
static int8_t    BitI1(uint8_t* p) { int8_t    val; memcpy(&val, p, 1); return val; }
static uint16_t  BitU2(uint8_t* p) { uint16_t  val; memcpy(&val, p, 2); return val; }
static int16_t   BitI2(uint8_t* p) { int16_t   val; memcpy(&val, p, 2); return val; }
static uint32_t  BitU4(uint8_t* p) { uint32_t  val; memcpy(&val, p, 4); return val; }
static int32_t   BitI4(uint8_t* p) { int32_t   val; memcpy(&val, p, 4); return val; }
static uint64_t  BitU8(uint8_t* p) { uint64_t  val; memcpy(&val, p, 8); return val; }
static int64_t   BitI8(uint8_t* p) { int64_t   val; memcpy(&val, p, 8); return val; }
static float     BitF4(uint8_t* p) { float     val; memcpy(&val, p, 4); return val; }
static double    BitF8(uint8_t* p) { double    val; memcpy(&val, p, 8); return val; }

/**
 * @brief Decode log buffer
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    Length Byte data                       | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|       playload data                       | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |

IPC : |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
Text: |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|             Log Text String               | CHECKSUM|

 * @param[in] decoder:ssr_stream decoder
 * @param[in] data: input stream data
 * @return: None
 */
static int32_t asensing_ssrstream_input(SSRDecoder_t* decoder, uint8_t data)
{
#if 0
  /* synchronize frame */
  uint32_t len = 0;
  uint8_t ck[2] = { 0 };
  if (decoder->length == 0) {
    if (decoder->nbyte == 0)
    {
      if (data != LOG_SYNC0)
      {
        return 0;
      }
      decoder->header[decoder->nbyte++] = data;
      return 0;
    }

    if (decoder->nbyte == 1)
    {
      if (data != LOG_SYNC1)
      {
        decoder->nbyte = 0;
        return 0;
      }
      decoder->header[decoder->nbyte++] = data;
      return 0;
    }

    if (decoder->nbyte == 4)
    {
      decoder->type_id = BitU2(decoder->header + 2);
      if (LOC_REPORT_ID_SSR_LOS_BLK != decoder->type_id) {
        decoder->nbyte = 0;
        return 0;
      }
    }

    decoder->header[decoder->nbyte++] = data;

    if (decoder->nbyte == SSR_STREAM_HEADER_LEN)
    {
      decoder->length = BitU4(decoder->header + 4);
      decoder->payload = (uint8_t*)OS_MALLOC(decoder->length + SSR_STREAM_CRC_LEN + SSR_STREAM_HEADER_LEN);
      memcpy(decoder->payload, decoder->header, SSR_STREAM_HEADER_LEN);
      decoder->nbyte = SSR_STREAM_HEADER_LEN;
    }
  }
  else {
    decoder->payload[decoder->nbyte++] = data;
    if (decoder->nbyte < decoder->length + SSR_STREAM_CRC_LEN + SSR_STREAM_HEADER_LEN) {
      return 0;
    }
    decoder->nbyte = 0;

    loc_checksum(decoder->payload + 2, 6 + decoder->length, &ck[0], &ck[1]);
    if (!((ck[0] == decoder->payload[decoder->length + 8]) &&
      (ck[1] == decoder->payload[decoder->length + 9])))
    {
      LOGE(TAG_SM, "CheckSum Fail\n");
      return 0;
    }
    else {
      decoder->length = decoder->length + SSR_STREAM_CRC_LEN + SSR_STREAM_HEADER_LEN;
      return decoder->type_id;
    }
  }
#endif
  return 0;
}

int32_t parse_ssr_stream(SSRDecoder_t* decode, uint8_t* data, uint32_t size)
{
  static uint32_t i = 0;
  /*int32_t ret = 0;*/
  for (; i < size; i++) 
  {
    (void)asensing_ssrstream_input(decode, data[i]);
   /* if (0 != ret) 
    {
      return ret;
    }*/
  }
  if (i == size) 
  {
    i = 0;
    return -1;
  }
  return 0;
}