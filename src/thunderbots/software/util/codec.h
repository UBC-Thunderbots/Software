#ifndef UTIL_CODEC_H
#define UTIL_CODEC_H

#include <cmath>
#include <cstddef>
#include <cstdint>

/**
 * \brief Encodes a floating-point number in IEEE754 single-precision format.
 *
 * \param[in] x the value to encode.
 *
 * \return the encoded form.
 */
uint32_t encode_float_to_u32(float x);

/**
 * \brief Decodes a floating-point number from IEEE754 single-precision format.
 *
 * \param[in] x the value to decode.
 *
 * \return the floating-point number.
 */
float decode_u32_to_float(uint32_t x);

/**
 * \brief Encodes a floating-point number in IEEE754 double-precision format.
 *
 * \param[in] x the value to encode.
 *
 * \return the encoded form.
 */
uint64_t encode_double_to_u64(double x);

/**
 * \brief Decodes a floating-point number from IEEE754 double-precision format.
 *
 * \param[in] x the value to decode.
 *
 * \return the floating-point number.
 */
double decode_u64_to_double(uint64_t x);

/**
 * \brief Encodes an 8-bit integer to a byte array in big endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u8_be(void *b, uint8_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = x;
}

/**
 * \brief Encodes a 16-bit integer to a byte array in big endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u16_be(void *b, uint16_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x >> 8);
    buf[1]       = static_cast<uint8_t>(x);
}

/**
 * \brief Encodes a 24-bit integer to a byte array in big endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u24_be(void *b, uint32_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x >> 16);
    buf[1]       = static_cast<uint8_t>(x >> 8);
    buf[2]       = static_cast<uint8_t>(x);
}

/**
 * \brief Encodes a 32-bit integer to a byte array in big endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u32_be(void *b, uint32_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x >> 24);
    buf[1]       = static_cast<uint8_t>(x >> 16);
    buf[2]       = static_cast<uint8_t>(x >> 8);
    buf[3]       = static_cast<uint8_t>(x);
}

/**
 * \brief Encodes a 64-bit integer to a byte array in big endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u64_be(void *b, uint64_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x >> 56);
    buf[1]       = static_cast<uint8_t>(x >> 48);
    buf[2]       = static_cast<uint8_t>(x >> 40);
    buf[3]       = static_cast<uint8_t>(x >> 32);
    buf[4]       = static_cast<uint8_t>(x >> 24);
    buf[5]       = static_cast<uint8_t>(x >> 16);
    buf[6]       = static_cast<uint8_t>(x >> 8);
    buf[7]       = static_cast<uint8_t>(x);
}

/**
 * \brief Encodes a single-precision floating-point number to a byte array in
 * big endian form.
 *
 * The floating-point number will consume 4 bytes of storage.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the floating-point number to encode.
 */
inline void encode_float_be(void *b, float x)
{
    encode_u32_be(b, encode_float_to_u32(x));
}

/**
 * \brief Encodes a double-precision floating-point number to a byte array in
 * big endian form.
 *
 * The floating-point number will consume 8 bytes of storage.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the floating-point number to encode.
 */
inline void encode_double_be(void *b, double x)
{
    encode_u64_be(b, encode_double_to_u64(x));
}

/**
 * \brief Extracts an 8-bit integer from a data buffer in big endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint8_t decode_u8_be(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    return buf[0];
}

/**
 * \brief Extracts a 16-bit integer from a data buffer in big endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint16_t decode_u16_be(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint16_t val       = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
        val = static_cast<uint16_t>((val << 8) | buf[i]);
    }
    return val;
}

/**
 * \brief Extracts a 24-bit integer from a data buffer in big endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint32_t decode_u24_be(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint32_t val       = 0;
    for (std::size_t i = 0; i < 3; ++i)
    {
        val <<= 8;
        val |= buf[i];
    }
    return val;
}

/**
 * \brief Extracts a 32-bit integer from a data buffer in big endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint32_t decode_u32_be(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint32_t val       = 0;
    for (std::size_t i = 0; i < 4; ++i)
    {
        val <<= 8;
        val |= buf[i];
    }
    return val;
}

/**
 * \brief Extracts a 64-bit integer from a data buffer in big endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint64_t decode_u64_be(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint64_t val       = 0;
    for (std::size_t i = 0; i < 8; ++i)
    {
        val <<= 8;
        val |= buf[i];
    }
    return val;
}

/**
 * \brief Extracts a single-precision floating-point number from a data buffer
 * in big endian form.
 *
 * The floating-point number must be 4 bytes wide.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the floating-point number.
 */
inline float decode_float_be(const void *buffer)
{
    return decode_u32_to_float(decode_u32_be(buffer));
}

/**
 * \brief Extracts a double-precision floating-point number from a data buffer
 * in big endian form.
 *
 * The floating-point number must be 8 bytes wide.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the floating-point number.
 */
inline double decode_double_be(const void *buffer)
{
    return decode_u64_to_double(decode_u64_be(buffer));
}

/**
 * \brief Encodes an 8-bit integer to a byte array in little endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u8_le(void *b, uint8_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = x;
}

/**
 * \brief Encodes a 16-bit integer to a byte array in little endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u16_le(void *b, uint16_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x);
    buf[1]       = static_cast<uint8_t>(x >> 8);
}

/**
 * \brief Encodes a 24-bit integer to a byte array in little endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u24_le(void *b, uint32_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x);
    buf[1]       = static_cast<uint8_t>(x >> 8);
    buf[2]       = static_cast<uint8_t>(x >> 16);
}

/**
 * \brief Encodes a 32-bit integer to a byte array in little endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u32_le(void *b, uint32_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x);
    buf[1]       = static_cast<uint8_t>(x >> 8);
    buf[2]       = static_cast<uint8_t>(x >> 16);
    buf[3]       = static_cast<uint8_t>(x >> 24);
}

/**
 * \brief Encodes a 64-bit integer to a byte array in little endian form.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the integer to encode.
 */
inline void encode_u64_le(void *b, uint64_t x)
{
    uint8_t *buf = static_cast<uint8_t *>(b);
    buf[0]       = static_cast<uint8_t>(x);
    buf[1]       = static_cast<uint8_t>(x >> 8);
    buf[2]       = static_cast<uint8_t>(x >> 16);
    buf[3]       = static_cast<uint8_t>(x >> 24);
    buf[4]       = static_cast<uint8_t>(x >> 32);
    buf[5]       = static_cast<uint8_t>(x >> 40);
    buf[6]       = static_cast<uint8_t>(x >> 48);
    buf[7]       = static_cast<uint8_t>(x >> 56);
}

/**
 * \brief Encodes a single-precision floating-point number to a byte array in
 * little endian form.
 *
 * The floating-point number will consume 4 bytes of storage.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the floating-point number to encode.
 */
inline void encode_float_le(void *b, float x)
{
    encode_u32_le(b, encode_float_to_u32(x));
}

/**
 * \brief Encodes a double-precision floating-point number to a byte array in
 * little endian form.
 *
 * The floating-point number will consume 8 bytes of storage.
 *
 * \param[out] b the buffer into which to encode.
 *
 * \param[in] x the floating-point number to encode.
 */
inline void encode_double_le(void *b, double x)
{
    encode_u64_le(b, encode_double_to_u64(x));
}

/**
 * \brief Extracts an 8-bit integer from a data buffer in little endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint8_t decode_u8_le(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    return buf[0];
}

/**
 * \brief Extracts a 16-bit integer from a data buffer in little endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint16_t decode_u16_le(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint16_t val       = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
        val = static_cast<uint16_t>((val << 8) | buf[1 - i]);
    }
    return val;
}

/**
 * \brief Extracts a 24-bit integer from a data buffer in little endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint32_t decode_u24_le(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint32_t val       = 0;
    for (std::size_t i = 0; i < 3; ++i)
    {
        val <<= 8;
        val |= buf[2 - i];
    }
    return val;
}

/**
 * \brief Extracts a 32-bit integer from a data buffer in little endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint32_t decode_u32_le(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint32_t val       = 0;
    for (std::size_t i = 0; i < 4; ++i)
    {
        val <<= 8;
        val |= buf[3 - i];
    }
    return val;
}

/**
 * \brief Extracts a 64-bit integer from a data buffer in little endian form.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the integer.
 */
inline uint64_t decode_u64_le(const void *buffer)
{
    const uint8_t *buf = static_cast<const uint8_t *>(buffer);
    uint64_t val       = 0;
    for (std::size_t i = 0; i < 8; ++i)
    {
        val <<= 8;
        val |= buf[7 - i];
    }
    return val;
}

/**
 * \brief Extracts a single-precision floating-point number from a data buffer
 * in little endian form.
 *
 * The floating-point number must be 4 bytes wide.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the floating-point number.
 */
inline float decode_float_le(const void *buffer)
{
    return decode_u32_to_float(decode_u32_le(buffer));
}

/**
 * \brief Extracts a double-precision floating-point number from a data buffer
 * in little endian form.
 *
 * The floating-point number must be 8 bytes wide.
 *
 * \param[in] buffer the data to extract from.
 *
 * \return the floating-point number.
 */
inline double decode_double_le(const void *buffer)
{
    return decode_u64_to_double(decode_u64_le(buffer));
}

#endif
