#pragma once

#include <cstdint>
#include <vector>

const uint8_t START_END_FLAG_BYTE = 0x00;

namespace
{
    /**
     * Generates a CRC-16 adhering to the CRC-16-CCITT standard
     *
     * @param data the data to calculate the crc of
     * @param length the length of the data
     * @return CRC-16 for given data and length
     */
    uint16_t crc16(const std::vector<uint8_t>& data, uint16_t length)
    {
        uint8_t x;
        uint16_t crc = 0xFFFF;
        int i        = 0;

        while (length--)
        {
            x = static_cast<uint8_t>(crc >> 8u) ^ data[i++];
            x ^= static_cast<uint8_t>(x >> 4u);
            crc = static_cast<uint16_t>((crc << 8u) ^ (x << 12u) ^ (x << 5u) ^ x);
        }

        return crc;
    }

    /**
     * Implementation of modified Consistent Overhead Byte Stuffing(COBS) encoding
     * https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
     *
     * Uses a Delimiter byte at both the beginning(different) and end(normal)
     *
     * @param to_encode the vector of bytes to encode
     * @return a vector of bytes encoded with COBS
     */
    std::vector<uint8_t> cobsEncoding(const std::vector<uint8_t>& to_encode)
    {
        auto encoded = std::vector<uint8_t>();
        encoded.emplace_back(START_END_FLAG_BYTE);

        size_t overhead_location = 1;
        uint8_t overhead         = 0x01;

        for (auto byte : to_encode)
        {
            if (byte == START_END_FLAG_BYTE)
            {
                encoded.insert(encoded.begin() + overhead_location, overhead);
                overhead_location = encoded.size();
                overhead          = 0x01;
            }
            else
            {
                encoded.emplace_back(byte);
                if (++overhead == 0xFF)
                {
                    encoded.insert(encoded.begin() + overhead_location, overhead);
                    overhead_location = encoded.size();
                    overhead          = 0x01;
                }
            }
        }
        encoded.insert(encoded.begin() + overhead_location, overhead);
        encoded.emplace_back(START_END_FLAG_BYTE);

        return encoded;
    }

    /**
     * Implementation of modified Consistent Overhead Byte Stuffing(COBS) decoding
     * https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
     *
     * Uses a Delimiter byte at both the beginning and end instead of a single
     * delimiter byte at the end as in the wikipedia example.
     *
     * @param to_decode the vector of bytes to decode
     * @param decoded the vector that bytes are decoded to
     * @return whether the decode was successful
     */
    bool cobsDecoding(
        const std::vector<uint8_t>& to_decode, std::vector<uint8_t>& decoded)
    {
        if (to_decode.front() != START_END_FLAG_BYTE ||
            to_decode.back() != START_END_FLAG_BYTE)
        {
            return false;
        }

        uint16_t i = 1;
        while (i < to_decode.size())
        {
            uint8_t overhead = to_decode[i++];
            // Check that overhead does not point to past the end of the vector
            if (overhead + i > to_decode.size())
            {
                return false;
            }
            // Check that instances of the START_END_FLAG_BYTE are not in the middle of
            // the vector
            if (overhead == START_END_FLAG_BYTE && i != to_decode.size())
            {
                return false;
            }
            for (uint16_t j = 1; i < to_decode.size() && j < overhead; j++)
            {
                decoded.emplace_back(to_decode[i++]);
            }
            if (overhead < 0xFF && i != to_decode.size() - 1 &&
                overhead != START_END_FLAG_BYTE)
            {
                decoded.emplace_back(START_END_FLAG_BYTE);
            }
        }

        return true;
    }
}  // anonymous namespace

/**
 * Acts as a frame for uart messages
 * The type of message should be structs.
 * These structs should not include pointers or references.
 *
 * @tparam T the type of message being framed
 */
template <typename T>
struct UartMessageFrame
{
    uint16_t length;
    uint16_t crc;
    T message;

    /**
     * Prepares a struct for being sent over uart.
     * Performs both the framing and encoding.
     *
     * @tparam T type of struct beginning sent
     * @param data struct being marshalled
     * @return vector of bytes to be sent over uart
     */
    std::vector<uint8_t> marshallUartPacket()
    {
        auto message_ptr = reinterpret_cast<const char*>(this);
        std::vector<uint8_t> framed_packet(message_ptr, message_ptr + sizeof(*this));
        return cobsEncoding(framed_packet);
    }

    /**
     * Checks if the length and crc of the UartMessageFrame match
     * the length and crc of the message
     * @tparam T type that is being wrapped in data
     * @param data the data to verify
     * @return true if the length and crc match false otherwise
     */
    bool verifyLengthAndCrc()
    {
        uint16_t expected_length = sizeof(message);
        auto ptr                 = reinterpret_cast<const uint8_t*>(&message);
        auto bytes               = std::vector<uint8_t>(ptr, ptr + sizeof(message));
        uint16_t expected_crc    = crc16(bytes, length);
        return crc == expected_crc && length == expected_length;
    }
};

/**
 * Calculates the length and crc for the given data and returns
 * a UartMessageFrame with the given data and computed fields
 *
 * @tparam T the type of the data param
 * @param data the data to put in a UartMessageFrame
 * @return a UartMessageFrame with the given data and computed fields
 */
template <typename T>
UartMessageFrame<T> createUartMessageFrame(const T& data)
{
    uint16_t length = sizeof(data);
    auto ptr        = reinterpret_cast<const uint8_t*>(&data);
    auto bytes      = std::vector<uint8_t>(ptr, ptr + sizeof(data));
    uint16_t crc    = crc16(bytes, length);
    UartMessageFrame<T> frame{length, crc, data};
    return frame;
}

/**
 * Converts a vector of bytes to its corresponding UartMessageFrame.
 *
 * @tparam T type of UartMessageFrame to unmarshal to
 * @param data vector of bytes to unmarshal
 * @param message_frame frame to unmarshal to
 * @return whether the unmarshall was successful
 *
 */
template <typename T>
bool unmarshalUartPacket(const std::vector<uint8_t>& data, UartMessageFrame<T>& message_frame)
{
    auto decoded = std::vector<uint8_t>();
    if (!cobsDecoding(data, decoded))
    {
        return false;
    }
    if (decoded.size() != sizeof(message_frame))
    {
        return false;
    }
    std::copy(decoded.begin(), decoded.end(),
              reinterpret_cast<uint8_t*>(&message_frame));
    return message_frame.verifyLengthAndCrc();
}

template <typename T>
size_t getMarshalledSize(const T& data) {
    auto data_ptr = reinterpret_cast<const char*>(&data);
    std::vector<uint8_t>data_vector (data_ptr, data_ptr + sizeof(data));
    return cobsEncoding(data_vector).size() + sizeof(UartMessageFrame<T>) - sizeof(T);
}
