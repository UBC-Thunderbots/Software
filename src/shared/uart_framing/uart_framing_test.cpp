#include "shared/uart_framing/uart_framing.hpp"

#include <gtest/gtest.h>
extern "C"
{
#include "proto/robot_status_msg.nanopb.h"
}

class CobsEncodingTest : public ::testing::TestWithParam<
                             std::tuple<std::vector<uint8_t>, std::vector<uint8_t>>>
{
};

// Returns 0x00 0x01 0x02 0x03 0x04 ... 0xFB 0xFC 0xFD 0xFE 0xFF
static std::vector<uint8_t> generateBigByteVectorToEncode()
{
    std::vector<uint8_t> data;
    for (uint8_t j = 0x00; j < 0xFF; j++)
    {
        data.emplace_back(j);
    }
    data.emplace_back(0xFF);
    return data;
}

// Returns 0x00 0x01 0xFF 0x01 0x02 ... 0xFD 0xFE 0x02 0xFF 0x00
static std::vector<uint8_t> generateBigByteVectorToDecode()
{
    std::vector<uint8_t> data = generateBigByteVectorToEncode();
    std::replace(data.begin(), data.end(), 0x00, 0xFF);
    // insert start and end bytes
    data.insert(data.begin(), START_END_FLAG_BYTE);
    data.emplace_back(START_END_FLAG_BYTE);

    // insert overhead byte
    data.insert(data.begin() + 1, 0x01);

    // insert end overhead byte
    data.insert(data.end() - 2, 0x02);

    return data;
}

TEST_P(CobsEncodingTest, encode_decode_test)
{
    // Check that cobs encodes to the expected value
    auto encoded = cobsEncoding(std::get<0>(GetParam()));
    EXPECT_EQ(encoded, std::get<1>(GetParam()));
    // Check that cobs decodes to the expected value
    auto decoded = std::vector<uint8_t>();
    EXPECT_TRUE(cobsDecoding(encoded, decoded));
    EXPECT_EQ(decoded, std::get<0>(GetParam()));
}

INSTANTIATE_TEST_CASE_P(
    encode_decode_test, CobsEncodingTest,
    ::testing::Values(
        std::make_tuple(std::vector<uint8_t>({0x01}),
                        std::vector<uint8_t>({START_END_FLAG_BYTE, 0x02, 0x01,
                                              START_END_FLAG_BYTE})),
        std::make_tuple(std::vector<uint8_t>({START_END_FLAG_BYTE}),
                        std::vector<uint8_t>({START_END_FLAG_BYTE, 0x01, 0x01,
                                              START_END_FLAG_BYTE})),
        std::make_tuple(std::vector<uint8_t>({0x01, 0x02, 0x03, 0x04}),
                        std::vector<uint8_t>({START_END_FLAG_BYTE, 0x05, 0x01, 0x02, 0x03,
                                              0x04, START_END_FLAG_BYTE})),
        std::make_tuple(std::vector<uint8_t>({START_END_FLAG_BYTE, START_END_FLAG_BYTE,
                                              START_END_FLAG_BYTE, START_END_FLAG_BYTE}),
                        std::vector<uint8_t>({START_END_FLAG_BYTE, 0x01, 0x01, 0x01, 0x01,
                                              0x01, START_END_FLAG_BYTE})),
        std::make_tuple(generateBigByteVectorToEncode(),
                        generateBigByteVectorToDecode())));

class CobsDecodingErrorTest : public ::testing::TestWithParam<std::vector<uint8_t>>
{
};

TEST_P(CobsDecodingErrorTest, decode_error_tests)
{
    auto decoded = std::vector<uint8_t>();
    EXPECT_FALSE(cobsDecoding(GetParam(), decoded));
}

INSTANTIATE_TEST_CASE_P(
    decode_error_tests, CobsDecodingErrorTest,
    ::testing::Values(
        std::vector<uint8_t>({0x01}), std::vector<uint8_t>({0x01, START_END_FLAG_BYTE}),
        std::vector<uint8_t>({0x01, START_END_FLAG_BYTE, 0x01}),
        std::vector<uint8_t>({START_END_FLAG_BYTE, 0x02, START_END_FLAG_BYTE}),
        std::vector<uint8_t>(100, START_END_FLAG_BYTE), std::vector<uint8_t>(100, 0xFF)));

class UartFramingTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        test_message = createNanoPbPowerControl(ChickerCommandMode::AUTOCHIPORKICK, 1.0,
                                                2.0, AutoChipOrKickMode::AUTOCHIP, 3.0,
                                                4.0, TbotsProto_Geneva_Slot_LEFT);
    }

    TbotsProto_PowerControl test_message;
    const uint16_t TEST_MESSAGE_CRC = 63160;  // From online calculator
};

bool operator==(const TbotsProto_PowerControl& lhs, const TbotsProto_PowerControl& rhs)
{
    return serializeToVector(lhs) == serializeToVector(rhs);
}

TEST_F(UartFramingTest, length_and_crc_test)
{
    auto test_frame = createUartFrame(test_message);
    // Check size of frame is the size of the original struct + length and crc fields
    EXPECT_EQ(sizeof(test_frame), sizeof(TbotsProto_PowerFrame));
    // Check fields of frame
    EXPECT_EQ(test_frame.length, TbotsProto_PowerControl_size);
    EXPECT_EQ(test_frame.crc, TEST_MESSAGE_CRC);
    EXPECT_EQ(test_frame.power_msg.power_control, test_message);
    EXPECT_TRUE(verifyLengthAndCrc(test_frame));
}

TEST_F(UartFramingTest, verify_length_and_crc_detect_wrong_length_and_crc_test)
{
    auto test_frame = createUartFrame(test_message);
    EXPECT_TRUE(verifyLengthAndCrc(test_frame));
    test_frame.length = 0;
    EXPECT_FALSE(verifyLengthAndCrc(test_frame));
    test_frame.length = sizeof(test_message);
    test_frame.crc    = 0;
    EXPECT_FALSE(verifyLengthAndCrc(test_frame));
}

TEST_F(UartFramingTest, marshalling_test)
{
    auto test_frame = createUartFrame(test_message);
    auto bytes      = marshallUartPacket(test_frame);
    EXPECT_EQ(bytes.front(), START_END_FLAG_BYTE);
    EXPECT_EQ(bytes.back(), START_END_FLAG_BYTE);
    // Check that size makes sense
    EXPECT_EQ(bytes.size(), TbotsProto_PowerFrame_size + 3 * sizeof(uint8_t));

    TbotsProto_PowerFrame test_frame_unmarshalled = TbotsProto_PowerFrame_init_default;
    EXPECT_TRUE(unmarshalUartPacket(bytes, test_frame_unmarshalled));
    // Check fields of frame
    EXPECT_EQ(test_frame_unmarshalled.length, TbotsProto_PowerControl_size);
    EXPECT_EQ(test_frame_unmarshalled.crc, TEST_MESSAGE_CRC);
    EXPECT_EQ(test_frame_unmarshalled.power_msg.power_control, test_message);
    EXPECT_TRUE(verifyLengthAndCrc(test_frame_unmarshalled));
}
