/**@file        ssr_stream_parse.h
 * @brief       SSR stream Decoder
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/07  <td>0.1      <td>zhanglei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SSR_STREAM_PARSE_H__
#define __SSR_STREAM_PARSE_H__

#include "cmn_def.h"
#include "gnss_type.h"


BEGIN_DECL

#define SSR_STREAM_HEADER_LEN (8)
#define SSR_STREAM_CRC_LEN    (2)

typedef struct {
  uint8_t  header[SSR_STREAM_HEADER_LEN];
  uint32_t type_id;
  uint32_t nbyte;
  uint32_t length;
  uint8_t* payload;
} SSRDecoder_t;

int32_t parse_ssr_stream(SSRDecoder_t* decode, uint8_t* data, uint32_t size);

END_DECL

#endif