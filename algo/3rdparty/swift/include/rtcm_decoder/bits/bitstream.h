#ifndef SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITSTREAM_H
#define SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITSTREAM_H

#include <rtcm_decoder/bits/bits.h>

namespace swift {
namespace rtcm_decoder {

struct BitStream {
  const uint8_t *data;
  uint32_t data_len = 0;
  uint32_t offset = 0;
};

/**
 * @brief Check if reading outside of buffer.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 *
 * @return true if requested length of data is smaller than the buffer length.
 */
static inline bool bitstream_would_overflow(const BitStream &buff,
                                            const uint32_t pos,
                                            const uint32_t len) {
  return (buff.offset + pos + len) > buff.data_len;
}

/**
 * @brief Initialize bitstream.
 *
 * @param payload Pointer to payload to initialize bitstream data with.
 * @param payoad_len Payload length.
 * @param buff Initialized bitstream.
 */
static inline void bitstream_init(const uint8_t *const payload,
                                  const uint32_t payload_len,
                                  BitStream *const buff) {
  buff->data = payload;
  buff->data_len = payload_len;
  buff->offset = 0;
}

/**
 * @brief Move forward position of bitstream.
 *
 * @param len Number of bits to seek.
 * @param buff Bitstream to move forward position of.
 */
static inline void bitstream_seek_next_pos(const uint32_t len,
                                           BitStream *const buff) {
  buff->offset += len;
}

/**
 * @brief Get 8 bit unsigned integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu8(const BitStream &buff, const uint32_t pos,
                                      const uint32_t len, uint8_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbitu8(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 8 bit unsigned integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu8_and_seek_next_pos(const uint32_t len,
                                                        BitStream *const buff,
                                                        uint8_t *const out) {
  const bool ret = bitstream_getbitu8(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get 16 bit unsigned integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu16(const BitStream &buff,
                                       const uint32_t pos, const uint32_t len,
                                       uint16_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbitu16(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 16 bit unsigned integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu16_and_seek_next_pos(const uint32_t len,
                                                         BitStream *const buff,
                                                         uint16_t *const out) {
  const bool ret = bitstream_getbitu16(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get 32 bit unsigned integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu32(const BitStream &buff,
                                       const uint32_t pos, const uint32_t len,
                                       uint32_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbitu32(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 32 bit unsigned integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitu32_and_seek_next_pos(const uint32_t len,
                                                         BitStream *const buff,
                                                         uint32_t *const out) {
  const bool ret = bitstream_getbitu32(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get 8 bit signed integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits8(const BitStream &buff, const uint32_t pos,
                                      const uint32_t len, int8_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbits8(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 8 bit signed integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits8_and_seek_next_pos(const uint32_t len,
                                                        BitStream *const buff,
                                                        int8_t *const out) {
  const bool ret = bitstream_getbits8(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get 16 bit signed integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits16(const BitStream &buff,
                                       const uint32_t pos, const uint32_t len,
                                       int16_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbits16(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 16 bit signed integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits16_and_seek_next_pos(const uint32_t len,
                                                         BitStream *const buff,
                                                         int16_t *const out) {
  const bool ret = bitstream_getbits16(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get 32 bit signed integer field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits32(const BitStream &buff,
                                       const uint32_t pos, const uint32_t len,
                                       int32_t *const out) {
  if (bitstream_would_overflow(buff, pos, len)) {
    return false;
  }
  *out = getbits32(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get 32 bit signed integer field from bitstream starting from the
 * offset. Also move the offset once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbits32_and_seek_next_pos(const uint32_t len,
                                                         BitStream *const buff,
                                                         int32_t *const out) {
  const bool ret = bitstream_getbits32(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

/**
 * @brief Get boolean field from bitstream.
 *
 * @param buff Bitstream to read data from.
 * @param pos Position in buffer of start of bit field in bits.
 * @param len Length of bit field in bits.
 * @param out Variable where the field value is saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitbool(const BitStream &buff,
                                        const uint32_t pos, const uint32_t len,
                                        bool *const out) {
  if (bitstream_would_overflow(buff, pos, len) || (len != 1)) {
    return false;
  }
  *out = getbitbool(buff.data, buff.offset + pos, len);
  return true;
}

/**
 * @brief Get bool from bitstream starting from the offset. Also move the offset
 * once read.
 *
 * @param len Length of bit field in bits.
 * @param buff Bitstream to read data from.
 * @param out Variable where the field value will be saved.
 *
 * @return true if extraction of bits was successful.
 */
static inline bool bitstream_getbitbool_and_seek_next_pos(const uint32_t len,
                                                          BitStream *const buff,
                                                          bool *const out) {
  const bool ret = bitstream_getbitbool(*buff, 0, len, out);
  if (ret) {
    bitstream_seek_next_pos(len, buff);
  }
  return ret;
}

}  // namespace rtcm_decoder
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_RTCM_DECODER_BITS_BITSTREAM_H
