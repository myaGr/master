/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_SBP_MSG_H
#define SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_SBP_MSG_H

#include <rtcm_decoder/bits/bitstream.h>
#include <rtcm_decoder/constants.h>
#include <cstring>

namespace swift {
namespace rtcm_decoder {

// Encodes a SBP message
struct SbpMsg {
  uint16_t msg_type = 0;
  uint16_t sender_id = 0;
  uint8_t payload_len = 0;
  uint32_t offset = 0;
  std::array<uint8_t, 255> payload;
};

template <typename T>
bool sbp_can_unpack(const SbpMsg &buff) {
  return (buff.payload_len - buff.offset) >= sizeof(T);
}

bool sbp_u16_decode(SbpMsg *buff, uint16_t *v);

bool sbp_s16_decode(SbpMsg *buff, int16_t *v);

bool sbp_u32_decode(SbpMsg *buff, uint32_t *v);

bool sbp_s32_decode(SbpMsg *buff, int32_t *v);

bool sbp_u64_decode(SbpMsg *buff, uint64_t *v);

bool sbp_u8_decode(SbpMsg *buff, uint8_t *v);

bool sbp_s8_decode(SbpMsg *buff, int8_t *v);

}  // namespace rtcm_decoder
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_SBP_MSG_H
