#pragma once

#include <pb_decode.h>

#include <cstdint>
#include <vector>

#ifdef PLATFORMIO_BUILD
#include <power_frame_msg_platformio.h>
#else  // PLATFORMIO_BUILD
#include "proto/message_translation/power_frame_msg.hpp"
extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}
#endif  // PLATFORMIO_BUILD

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
     * Checks if the length and crc of the TbotsProto_PowerFrame match
     * the length and crc of the message
     *
     * @param frame the frame to verify
     * @return true if the length and crc match false otherwise
     */
    bool verifyLengthAndCrc(const TbotsProto_PowerFrame& frame)
    {
        uint16_t expected_length;
        std::vector<uint8_t> bytes;
        switch (frame.which_power_msg)
        {
            case TbotsProto_PowerFrame_power_control_tag:
                expected_length = TbotsProto_PowerPulseControl_size;
                bytes           = serializeToVector(frame.power_msg.power_control);
                break;
            case TbotsProto_PowerFrame_power_status_tag:
                expected_length = TbotsProto_PowerStatus_size;
                bytes           = serializeToVector(frame.power_msg.power_status);
                break;
            default:
                return false;
        }
        uint16_t expected_crc = crc16(bytes, static_cast<uint16_t>(frame.length));
        return frame.crc == expected_crc && frame.length == expected_length;
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
    bool cobsDecoding(const std::vector<uint8_t>& to_decode,
                      std::vector<uint8_t>& decoded)
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
 * Calculates the length and crc for the given data and returns
 * a TbotsProto_PowerFrame with the given data and computed fields
 *
 * @tparam T the type of power_msg
 * @param power_msg the power_msg to put in a UartMessageFrame
 * @return a TbotsProto_PowerFrame with the given data and computed fields
 */
template <typename T>
TbotsProto_PowerFrame inline createUartFrame(const T& power_msg)
{
    TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
    auto buffer                 = serializeToVector(power_msg);
    frame.length                = static_cast<uint32_t>(buffer.size());
    frame.crc                   = crc16(buffer, static_cast<uint16_t>(frame.length));
    setPowerMsg(frame, power_msg);
    return frame;
}

/**
 * Prepares a struct for being sent over uart.
 * Performs both the framing and encoding.
 *
 * @param frame frame being marshalled
 * @return vector of bytes to be sent over uart
 */
std::vector<uint8_t> inline marshallUartPacket(const TbotsProto_PowerFrame& frame)
{
    auto bytes = serializeToVector(frame);
    return cobsEncoding(bytes);
}

/**
 * Converts a vector of bytes to its corresponding TbotsProto_PowerFrame.
 *
 * @param data vector of bytes to unmarshal
 * @param message_frame frame to unmarshal to
 * @return whether the unmarshal was successful
 *
 */
bool inline unmarshalUartPacket(const std::vector<uint8_t>& data,
                                TbotsProto_PowerFrame& frame)
{
    std::vector<uint8_t> decoded;
    if (!cobsDecoding(data, decoded))
    {
        return false;
    }
    if (decoded.size() != TbotsProto_PowerFrame_size)
    {
        return false;
    }
    frame = TbotsProto_PowerFrame_init_default;
    pb_istream_t stream =
        pb_istream_from_buffer(static_cast<uint8_t*>(decoded.data()), decoded.size());
    if (!pb_decode(&stream, TbotsProto_PowerFrame_fields, &frame))
    {
        return false;
    }
    return verifyLengthAndCrc(frame);
}
/**
 * Finds the expected size of a TbotsProto_PowerStatus/TbotsProto_PowerPulseControl msg
 * once its encoded with cobs
 *
 * @tparam T type of power_msg should be TbotsProto_PowerStatus or
 * TbotsProto_PowerPulseControl
 * @param power_msg nanopb msg to get the marshalled size of
 * @return the expected marshalled size of the given message
 */
template <typename T>
size_t inline getMarshalledSize(const T& power_msg)
{
    auto frame  = createUartFrame(power_msg);
    auto buffer = serializeToVector(frame);
    return cobsEncoding(buffer).size();
}
