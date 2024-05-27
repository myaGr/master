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

#ifndef SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITS_H
#define SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITS_H

#include <cstdint>

namespace swift {
namespace rtcm_decoder {

/**
 * @brief Get bit field from buffer as an unsigned integer of 8 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as an 8 bit unsigned integer.
 */
uint8_t getbitu8(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as an unsigned integer of 16 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as a 16 bit unsigned integer.
 */
uint16_t getbitu16(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as an unsigned integer of 32 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as a 32 bit unsigned integer.
 */
uint32_t getbitu32(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as a signed integer of 8 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as an 8 bit signed integer.
 */
int8_t getbits8(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as a signed integer of 16 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as an 16 bit signed integer.
 */
int16_t getbits16(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as a signed integer of 32 bits.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as a 32 bit signed integer.
 */
int32_t getbits32(const uint8_t *buff, const uint32_t pos, const uint32_t len);

/**
 * @brief Get bit field from buffer as a boolean.
 * @details Unpacks `len` bits at bit position `pos` from the start of the
 * buffer. Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * @param buff The buffer to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @return Bit field as a boolean.
 */
bool getbitbool(const uint8_t *buff, const uint32_t pos, const uint32_t len);

}  // namespace rtcm_decoder
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITS_H
