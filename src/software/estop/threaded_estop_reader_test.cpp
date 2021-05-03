#include "threaded_estop_reader.h"

#include <gmock/gmock.h>

#include "gtest/gtest.h"
#include "mock_uart_communication.h"
#include "uart_communication.h"

static constexpr unsigned char STOP = 0;
static constexpr unsigned char PLAY = 1;

using ::testing::_;
using ::testing::at least;
using ::testing::Return;

TEST(ThreadedEstopReaderTest, estop_play_is_false_by_default_before_startup)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> ret_val(1, PLAY);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));


    int startup_interval_ms = 500;
    int tick_interval_ms    = 50;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);

    EXPECT_FALSE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest, estop_tick_is_called_every_interval)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    auto mock_uart_ptr = mock_uart.get();

    int startup_interval_ms  = 5;
    int tick_interval_ms     = 10;
    int thread_sleep_time_ms = 50;

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .Times(at least(thread_sleep_time_ms / tick_interval_ms))
        .WillRepeatedly(Return(stop_ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_time_ms));
}

TEST(ThreadedEstopReaderTest, estop_state_changes_based_on_read_val)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    int startup_interval_ms = 1;
    int tick_interval_ms    = 5;

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));


    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(startup_interval_ms + tick_interval_ms + 1));
    EXPECT_TRUE(estopReader.isEstopPlay());

    // change uart return value to stop
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));
    EXPECT_FALSE(estopReader.isEstopPlay());


    // change uart return value back to play
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));
    EXPECT_TRUE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest, estop_play_is_false_after_reading_unexpected_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));


    int startup_interval_ms = 0;
    int tick_interval_ms    = 2;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);


    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(garbage_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));

    EXPECT_FALSE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest, estop_state_does_not_change_after_long_periods)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));


    int startup_interval_ms = 0;
    int tick_interval_ms    = 2;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms * 10));
    EXPECT_TRUE(estopReader.isEstopPlay());

    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms * 10));

    EXPECT_FALSE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest,
     estop_play_is_true_after_reading_play_message_following_multiple_unexpected_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    int startup_interval_ms = 5;
    int tick_interval_ms    = 10;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms,
                                    tick_interval_ms);

    EXPECT_FALSE(estopReader.isEstopPlay());

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillRepeatedly(Return(play_ret_val));

    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms * 5));

    EXPECT_TRUE(estopReader.isEstopPlay());
}
