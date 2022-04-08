#include "shared/uart_framing/uart_framing.hpp"

#include <gtest/gtest.h>

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
    struct __attribute__((__packed__)) TestMessage
    {
        uint8_t a;
        uint16_t b;
        uint32_t c;
        uint64_t d;
        char e;

       public:
        bool operator==(const TestMessage& rhs) const
        {
            return a == rhs.a && b == rhs.b && c == rhs.c && d == rhs.d && e == rhs.e;
        }
    };

    void SetUp() override
    {
        test_message = {1, 2, 3, 4, 'e'};
    }

    TestMessage test_message;
    const uint16_t TEST_MESSAGE_CRC = 57534;  // From online calculator
};

TEST_F(UartFramingTest, length_and_crc_test)
{
    auto test_frame = createUartMessageFrame(test_message);
    // Check size of frame is the size of the original struct + length and crc fields
    EXPECT_EQ(sizeof(test_frame), sizeof(TestMessage) + 2 * sizeof(uint16_t));
    // Check fields of frame
    EXPECT_EQ(test_frame.length, sizeof(TestMessage));
    EXPECT_EQ(test_frame.crc, TEST_MESSAGE_CRC);
    EXPECT_EQ(test_frame.message, test_message);
    EXPECT_TRUE(test_frame.verifyLengthAndCrc());
}

TEST_F(UartFramingTest, verify_length_and_crc_detect_wrong_length_and_crc_test)
{
    UartMessageFrame<TestMessage> test_frame = {sizeof(test_message), TEST_MESSAGE_CRC,
                                                test_message};
    EXPECT_TRUE(test_frame.verifyLengthAndCrc());
    test_frame.length = 0;
    EXPECT_FALSE(test_frame.verifyLengthAndCrc());
    test_frame.length = sizeof(test_message);
    test_frame.crc    = 0;
    EXPECT_FALSE(test_frame.verifyLengthAndCrc());
}

TEST_F(UartFramingTest, verify_length_and_crc_detect_byte_error_test)
{
    // Convert original message to vector of bytes
    auto test_message_ptr = reinterpret_cast<uint8_t*>(&test_message);
    std::vector<uint8_t> bytes(test_message_ptr,
                               test_message_ptr + sizeof(test_message_ptr));
    // Add byte error
    bytes[2] = 0xFF;
    // Convert back to TestMessage
    TestMessage test_message_byte_error;
    std::copy(bytes.begin(), bytes.end(),
              reinterpret_cast<uint8_t*>(&test_message_byte_error));
    UartMessageFrame<TestMessage> test_frame = {sizeof(test_message), TEST_MESSAGE_CRC,
                                                test_message_byte_error};
    EXPECT_FALSE(test_frame.verifyLengthAndCrc());
}

TEST_F(UartFramingTest, marshalling_test)
{
    // bytes is expected to be of form 0x00 0x02 0x10 0x05 ... 0x65 0x00
    auto test_frame = createUartMessageFrame(test_message);
    auto bytes      = test_frame.marshallUartPacket();
    EXPECT_EQ(bytes.front(), START_END_FLAG_BYTE);
    EXPECT_EQ(bytes.back(), START_END_FLAG_BYTE);
    // Check overhead byte
    EXPECT_EQ(bytes[1], 0x02);
    // Check second last byte 'e'
    EXPECT_EQ(bytes.end()[-2], 0x65);
    // Check that size makes sense
    EXPECT_EQ(bytes.size(),
              sizeof(test_message) + 2 * sizeof(uint16_t) + 3 * sizeof(uint8_t));

    auto test_frame_unmarshalled = UartMessageFrame<TestMessage>();
    EXPECT_TRUE(unmarshalUartPacket<TestMessage>(bytes, test_frame_unmarshalled));
    // Check size of frame is the size of the original struct + length and crc fields
    EXPECT_EQ(sizeof(test_frame_unmarshalled),
              sizeof(TestMessage) + 2 * sizeof(uint16_t));
    // Check fields of frame
    EXPECT_EQ(test_frame_unmarshalled.length, sizeof(TestMessage));
    EXPECT_EQ(test_frame_unmarshalled.crc, TEST_MESSAGE_CRC);
    EXPECT_EQ(test_frame_unmarshalled.message, test_message);
    EXPECT_TRUE(test_frame_unmarshalled.verifyLengthAndCrc());
}
