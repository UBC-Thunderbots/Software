#pragma once

#include <cstdint>
#include <vector>
#include <optional>

const uint8_t START_END_FLAG_BYTE = 0x00;

/**
 * Acts as a frame for uart messages
 * The type of message should be structs.
 * These structs should not include pointers or references.
 *
 * @tparam T the type of message being framed
 */
template <typename T>
struct UartMessageFrame {
    uint16_t length;
    uint16_t crc;
    T message;
};

namespace {
    /**
     * Generates a CRC-16 adhering to the CRC-16-CCITT standard
     *
     * @param data the data to calculate the crc of
     * @param length the length of the data
     * @return CRC-16 for given data and length
     */
    template <typename T>
    uint16_t crc16(const T& data, uint16_t length)
    {
        uint8_t x;
        uint16_t crc = 0xFFFF;
        auto ptr = reinterpret_cast<const uint8_t*>(&data);
        auto bytes = std::vector<uint8_t>(ptr, ptr + sizeof(data));
        int i = 0;

        while(length--) {
            x = static_cast<uint8_t>(crc >> 8u) ^ bytes[i++];
            x ^= static_cast<uint8_t>(x >> 4u);
            crc = static_cast<uint16_t>((crc << 8u) ^ (x << 12u) ^ (x << 5u) ^ x);
        }

        return crc;
    }
    /**
     * Calculates the length and crc for the given data and returns
     * a UartMessageFrame with the given data and computed fields
     *
     * @tparam T the type of the data param
     * @param data the data to put in a UartMessageFrame
     * @return a UartMessageFrame with the given data and computed fields
     */
    template <typename T>
    UartMessageFrame<T> addLengthAndCrc(const T& data)
    {
        uint16_t length = sizeof(data);
        uint16_t crc = crc16(data, length);
        UartMessageFrame<T> frame {length, crc, data};
        return frame;
    }

    /**
     * Checks if the length and crc of the UartMessageFrame match
     * the length and crc of the message
     * @tparam T type that is being wrapped in data
     * @param data the data to verify
     * @return true if the length and crc match false otherwise
     */
    template <typename T>
    bool verifyLengthAndCrc(UartMessageFrame<T> data)
    {
        uint16_t expected_length = sizeof(data.message);
        uint16_t expected_crc = crc16(data.message, data.length);
        return data.crc == expected_crc && data.length == expected_length;
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
        uint8_t overhead = 0x01;

        for (auto byte: to_encode) {
            if (byte == START_END_FLAG_BYTE) {
                encoded.insert(encoded.begin() + overhead_location, overhead);
                overhead_location = encoded.size();
                overhead = 0x01;
            } else {
                encoded.emplace_back(byte);
                if (++overhead == 0xFF) {
                    encoded.insert(encoded.begin() + overhead_location, overhead);
                    overhead_location = encoded.size();
                    overhead = 0x01;
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
     * Uses a Delimiter byte at both the beginning(different) and end(normal)
     *
     * @param to_decode the vector of bytes to decode
     * @return a vector of bytes decoded with COBS
     */
    std::optional<std::vector<uint8_t>> cobsDecoding(const std::vector<uint8_t>& to_decode)
    {
        if (to_decode.front() != START_END_FLAG_BYTE || to_decode.back() != START_END_FLAG_BYTE) {
            return std::nullopt;
        }
        auto decoded = std::vector<uint8_t>();

        uint16_t i = 1;
        while (i < to_decode.size()) {
            uint8_t overhead = to_decode[i++];
            for (uint16_t j = 1; i < to_decode.size() && j < overhead; j++) {
                decoded.emplace_back(to_decode[i++]);
            }
            if (overhead < 0xFF && i != to_decode.size() - 1 && overhead != START_END_FLAG_BYTE) {
                decoded.emplace_back(START_END_FLAG_BYTE);
            }
        }
        // Make sure decoded message is within the upper bound of bytes
        if (decoded.size() + 3 + decoded.size() / 254 < to_decode.size()) {
            return std::nullopt;
        }
        return decoded;
    }
} // anonymous namespace

/**
 * Prepares a struct for being sent over uart.
 * Performs both the framing and encoding.
 *
 * @tparam T type of struct beginning sent
 * @param data struct being marshalled
 * @return vector of bytes to be sent over uart
 */
template <typename T>
std::vector<uint8_t> marshallUartPacket(const T& data)
{
    auto framed_packet_struct = addLengthAndCrc(data);
    auto framed_packet_ptr = reinterpret_cast<const char*>(&framed_packet_struct);
    std::vector<uint8_t> framed_packet (framed_packet_ptr, framed_packet_ptr + sizeof(framed_packet_struct));
    return cobsEncoding(framed_packet);
}

/**
 * Converts a vector of bytes to its corresponding UartMessageFrame if it exists.
 * Returns std::nullopt if bit or length errors are detected while decoding or through the crc.
 *
 * @tparam T type of UartMessageFrame to unmarshal to
 * @param data vector of bytes to unmarshal
 * @return a corresponding UartMessageFrame, if it exists
 *
 */
template <typename T>
std::optional<UartMessageFrame<T>> unmarshalUartPacket(const std::vector<uint8_t>& data)
{
    auto decoded_data = cobsDecoding(data);
    if (!decoded_data.has_value()) {
        return std::nullopt;
    }
    UartMessageFrame<T> message_frame;
    if (decoded_data->size() != sizeof(message_frame)) {
        return std::nullopt;
    }
    std::copy(decoded_data->begin(), decoded_data->end(), reinterpret_cast<uint8_t*>(&message_frame));
    if (verifyLengthAndCrc(message_frame)) {
        return message_frame;
    } else {
        return std::nullopt;
    }
}