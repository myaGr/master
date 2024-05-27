/**@file        pb_ag_parse.c
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

#include "cmn_utils.h"
#include "ag_log_parse.h"
#include "ipc_parse.h"
#include "mw_logcmprs.h"

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

static uint8_t u_ag_log_decmprs_data[LOG_DECODER_SIZE];




/**
 * @brief Decode log buffer
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    Length Byte data                       | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|       playload data                       | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |

IPC : |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
Text: |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|             Log Text String               | CHECKSUM|

 * @param[in] ipc
 * @return: None
 */
static int8_t ag_log_decode_frame(log_decoder_t* decoder, uint8_t ch, uint8_t* p_data, uint32_t* length)
{
  uint8_t cur_ch = ch;
  
  /* synchronize frame */
  if (decoder->nbyte == 0) 
  {
    if (!((LOG_SYNC0 == decoder->prev_ch) && (LOG_SYNC1 == cur_ch)))
    {
      decoder->prev_ch = cur_ch;
      return 0;
    }
    decoder->u_input_buf[0] = decoder->prev_ch;
    decoder->u_input_buf[1] = cur_ch;
    decoder->nbyte = 2;
    return 0;
  }

  if (decoder->nbyte > sizeof(decoder->u_input_buf))
  {
    decoder->nbyte = 0;
    memset(decoder->u_input_buf, 0, sizeof(decoder->u_input_buf));
    return 0;
  }
  decoder->u_input_buf[decoder->nbyte++] = ch;

  /* First 8 Bytes buffer check */
  if (decoder->nbyte < 8)
  {
    return 0;
  }

  if (decoder->nbyte == 8) 
  {
    uint32_t q_decode_length = BitU4(decoder->u_input_buf + 4);
    if (q_decode_length < *length)
    {
      decoder->length = q_decode_length;
      decoder->w_type = BitU2(decoder->u_input_buf + 2);
    }
    else
    {
      decoder->nbyte = 0;
      decoder->length = 0;
    }    
  }

  uint8_t b_PassCheck = TRUE;

  if ((LOG_TYPE_TEXT == decoder->w_type) || (LOG_TYPE_CRC_TEXT == decoder->w_type))
  {
    /* TODO: Add text message check if necessary */
  }
  else if ((LOG_TYPE_IPC == decoder->w_type) || (LOG_TYPE_CRC_IPC == decoder->w_type))
  {
    /* TODO: Add ipc message check if necessary */
  }
  else if ((LOG_TYPE_PACKAGE == decoder->w_type) || (LOG_TYPE_CRC_PACKAGE == decoder->w_type))
  {
    if (decoder->nbyte < 10)
    {
      return 0;
    }

    if (decoder->nbyte == 10)
    {
      LogPackageIdType w_PackageId = BitU2(decoder->u_input_buf + 8);
      switch (w_PackageId)
      {
      case LOG_PACKAGE_ID_MONITOR:
        /* Check length */
        break;
      case LOG_PACKAGE_ID_LOC_CONFIG:
        /* Check length */
        break;
      case LOG_PACKAGE_ID_INS_RESULT:
        /* Check length */
        break;
      case LOG_PACKAGE_ID_TSX_CORR:
        /* Check length */
        break;
      default:
        b_PassCheck = FALSE;
        break;
      }
    }
  }
  else if ((LOG_TYPE_CMPRS == decoder->w_type) || (LOG_TYPE_CRC_CMPRS == decoder->w_type))
  {
    /* Compressed message length max length is 25 * 1024, and min length is 5 * 1024 */
    if ((decoder->length > 25600) || (decoder->length < 5120))
    {
      b_PassCheck = FALSE;
    }
  }
  else
  {
    b_PassCheck = FALSE;
  }

  if (FALSE == b_PassCheck)
  {
    decoder->w_type = 0;
    decoder->nbyte = 0;
    decoder->length = 0;
    return 0;
  }

  if ((decoder->w_type >= LOG_TYPE_TEXT) && (decoder->w_type <= LOG_TYPE_CMPRS))
  {
    if (decoder->nbyte < decoder->length + 10)
    {
      return 0;
    }
    decoder->nbyte = 0;

    if (LOG_TYPE_CMPRS != decoder->w_type)
    {
      uint8_t ck[2] = { 0 };
      loc_checksum((uint8_t*)decoder->u_input_buf + 2, 6 + decoder->length, &ck[0], &ck[1]);
      if ((ck[0] != decoder->u_input_buf[decoder->length + 8]) ||
          (ck[1] != decoder->u_input_buf[decoder->length + 9]))
      {
        printf("Log Decode error, checksum error\n");
        return -2;
      }
    }
  }
  else if ((decoder->w_type >= LOG_TYPE_CRC_TEXT) && (decoder->w_type <= LOG_TYPE_CRC_CMPRS))
  {
    if (decoder->nbyte < decoder->length + 12)
    {
      return 0;
    }
    decoder->nbyte = 0;

    if (LOG_TYPE_CRC_CMPRS != decoder->w_type)
    {
      uint32_t q_Crc = 0;
      q_Crc = loc_crc24q((uint8_t*)decoder->u_input_buf + 2, 6 + decoder->length);

      uint32_t* p_crc24 = (uint32_t*)(&decoder->u_input_buf[8 + decoder->length]);

      if (q_Crc != *p_crc24)
      {
        return -2;
      }
    }
  }
  else
  {
    decoder->nbyte = 0;
    return 0;
  }

  if (*length < decoder->length)
  {
    return -1;
  }

  /* decoder->length + 2 is because of buffer has two bytes checksum */
  memcpy(p_data, decoder->u_input_buf + 8, decoder->length);
  *length = decoder->length;
  return (int8_t)decoder->w_type;
}

/**
 * @brief discard firtst 1 byte, re decode again
 * @param[in,out] decoder 
 * @param[in,out] p_data 
 * @param[in,out] length 
 * @return 
*/
static int8_t ag_log_decode_fixed(log_decoder_t* decoder, uint8_t* p_data, uint32_t* length)
{
  int8_t ret = -1;
  uint8_t stash_buf[LOG_DECODER_SIZE] = {0};
  uint32_t stash_length = decoder->length + 9;  // 10(1+1+2+4+2) -1
  memmove(stash_buf, decoder->u_input_buf + 1, stash_length);
  stash_buf[stash_length] = '\0';
  memset(decoder, 0, sizeof(log_decoder_t));
  decoder->u_decoded_length = LOG_DECODER_SIZE - LOG_PRECHECK_LEN;
  for (uint32_t j = 0; j < stash_length; j++)
  {
    ret = ag_log_decode_frame(decoder, stash_buf[j], p_data, length);
    if (ret == 0)
    {
      continue;
    }
    decoder->u_decoded_length = LOG_DECODER_SIZE - LOG_PRECHECK_LEN;
  }
  return ret;
}

/**
 * @brief Decode log buffer
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    Length Byte data                       | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|       playload data                       | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |

IPC : |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
Text: |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|             Log Text String               | CHECKSUM|

 * @param[in] ipc
 * @return: None
 */
int8_t ag_log_decode(log_decoder_t* decoder, uint8_t ch, uint8_t* p_data, uint32_t* length)
{
  int8_t ret = -1;
  ret  = ag_log_decode_frame(decoder, ch, p_data, length);
  if(-2 == ret)
  {
    ret = ag_log_decode_fixed(decoder, p_data, length);
  }
  return ret;
}

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
  uint8_t* p_data, uint32_t q_length, uint8_t b_compress_enable)
{
  if (NULL == p_hander)
  {
    return;
  }

  pz_log_decoder->u_decoded_length = LOG_DECODER_SIZE - LOG_PRECHECK_LEN;

  int8_t decoded_type = 0;

  for (uint32_t i = 0; i < q_length; i++)
  {
    decoded_type = ag_log_decode(pz_log_decoder, p_data[i], pz_log_decoder->u_decoded_buf, &pz_log_decoder->u_decoded_length);

    if (0 == decoded_type)
    {
      continue;
    }
    else if (-1 == decoded_type)
    {
      printf("Log Decode error, buffer length %d < %d\n",
        pz_log_decoder->u_decoded_length, q_length);
    }
    else if (-2 == decoded_type)
    {
      printf("Log Decode error, checksum error \n");
    }

    if ((LOG_TYPE_IPC     == decoded_type) || (LOG_TYPE_CRC_IPC     == decoded_type) ||
        (LOG_TYPE_TEXT    == decoded_type) || (LOG_TYPE_CRC_TEXT    == decoded_type) ||
        (LOG_TYPE_PACKAGE == decoded_type) || (LOG_TYPE_CRC_PACKAGE == decoded_type))
    {
      (p_hander)(decoded_type, pz_log_decoder->u_decoded_buf, pz_log_decoder->u_decoded_length);
    }
    else if ((LOG_TYPE_CMPRS == decoded_type) || (LOG_TYPE_CRC_CMPRS == decoded_type))
    {
      if (TRUE == b_compress_enable)
      {
        memset(u_ag_log_decmprs_data, 0, sizeof(u_ag_log_decmprs_data));
        uint32_t decmprs_length = mw_Decompress(pz_log_decoder->u_decoded_buf, u_ag_log_decmprs_data);
        loc_core_log_parse((pz_log_decoder + 1), p_hander, u_ag_log_decmprs_data, decmprs_length, FALSE);
      }
    }
    pz_log_decoder->u_decoded_length = LOG_DECODER_SIZE - LOG_PRECHECK_LEN;
  }

  return;
}
