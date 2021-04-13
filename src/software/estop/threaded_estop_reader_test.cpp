#include "threaded_estop_reader.h"

#include <gmock/gmock.h>

#include "gtest/gtest.h"
#include "mock_uart_communication.h"
#include "uart_communication.h"

static constexpr unsigned char STOP = 0;
static constexpr unsigned char PLAY = 1;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;

TEST(ThreadedEstopReaderTest, estop_state_returns_stop_by_default_before_startup)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> ret_val(1, PLAY);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(ret_val));

    int startup_interval_ms = 500;
    int tick_interval_ms    = 50;

    ThreadedEstopReader estopReader(startup_interval_ms, tick_interval_ms,
                                    std::move(mock_uart));

    EstopState estop_state = estopReader.getEstopState();

    EXPECT_EQ(estop_state, EstopState::STOP);
}

TEST(ThreadedEstopReaderTest, estop_state_changes_state_based_on_read_val)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    int startup_interval_ms = 0;
    int tick_interval_ms    = 5;

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    ThreadedEstopReader estopReader(startup_interval_ms, tick_interval_ms,
                                    std::move(mock_uart));

    std::this_thread::sleep_for(
        std::chrono::milliseconds(startup_interval_ms + tick_interval_ms + 1));
    EstopState estop_state = estopReader.getEstopState();
    EXPECT_EQ(estop_state, EstopState::PLAY);

    // change uart return value to stop
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));
    estop_state = estopReader.getEstopState();
    EXPECT_EQ(estop_state, EstopState::STOP);

    // change uart return value back to play
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));
    estop_state = estopReader.getEstopState();
    EXPECT_EQ(estop_state, EstopState::PLAY);
}

TEST(ThreadedEstopReaderTest, estop_state_is_error_after_reading_unexpected_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    int startup_interval_ms = 0;
    int tick_interval_ms    = 2;

    ThreadedEstopReader estopReader(startup_interval_ms, tick_interval_ms,
                                    std::move(mock_uart));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(garbage_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));

    EstopState estop_state = estopReader.getEstopState();

    EXPECT_EQ(estop_state, EstopState::STATUS_ERROR);
}

TEST(ThreadedEstopReaderTest, estop_state_does_not_change_after_long_periods)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    int startup_interval_ms = 0;
    int tick_interval_ms    = 2;

    ThreadedEstopReader estopReader(startup_interval_ms, tick_interval_ms,
                                    std::move(mock_uart));

    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms * 2));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(garbage_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms + 1));

    EstopState estop_state = estopReader.getEstopState();

    EXPECT_EQ(estop_state, EstopState::STATUS_ERROR);
}
