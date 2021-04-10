#include <gmock/gmock.h>
#include "gtest/gtest.h"

#include "uart_communication.h"
#include "mock_uart_communication.h"
#include "estop_manager.h"

static constexpr unsigned char STOP = 0;
static constexpr unsigned char PLAY = 1;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;

TEST(EstopManagerTest, is_play_state_returns_true_and_no_polling_by_default)
{
    EstopManager estopManager;

    EXPECT_TRUE(estopManager.isEstopStatePlay());
    EXPECT_FALSE(estopManager.isEstopPolling());
}

TEST(EstopManagerTest, is_play_state_is_false_for_polling_startup_and_changes_to_play_after_interval)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1,PLAY);
    std::vector<unsigned char> stop_ret_val(1,STOP);

    int startup_interval_ms = 50;
    int tick_interval_ms = 5;

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    EstopManager estopManager;
    estopManager.startEstopContinousPolling(startup_interval_ms, tick_interval_ms, std::move(mock_uart));
    EXPECT_FALSE(estopManager.isEstopStatePlay());
    EXPECT_TRUE(estopManager.isEstopPolling());

    std::this_thread::sleep_for(std::chrono::milliseconds(startup_interval_ms+tick_interval_ms+1));
    EXPECT_TRUE(estopManager.isEstopStatePlay());
}

TEST(EstopManagerTest, is_play_state_is_changes_based_on_play_and_stop_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1,PLAY);
    std::vector<unsigned char> stop_ret_val(1,STOP);

    int startup_interval_ms = 0;
    int tick_interval_ms = 5;

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    EstopManager estopManager;

    estopManager.startEstopContinousPolling(startup_interval_ms, startup_interval_ms, std::move(mock_uart));

    std::this_thread::sleep_for(std::chrono::milliseconds(startup_interval_ms+tick_interval_ms+1));

    //change uart return value to stop
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms+1));
    EXPECT_FALSE(estopManager.isEstopStatePlay());

    //change uart return value back to play
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms+1));
    EXPECT_TRUE(estopManager.isEstopStatePlay());

    //change uart return value to stop
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(stop_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms+1));
    EXPECT_FALSE(estopManager.isEstopStatePlay());
}

TEST(EstopManagerTest, is_play_state_is_false_after_reading_unexpected_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    unsigned char arbitrary_garbage_val = 'a';
    std::vector<unsigned char> play_ret_val(1,PLAY);
    std::vector<unsigned char> garbage_ret_val(1,arbitrary_garbage_val);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(play_ret_val));

    int startup_interval_ms = 0;
    int tick_interval_ms = 5;

    EstopManager estopManager;
    estopManager.startEstopContinousPolling(startup_interval_ms, startup_interval_ms, std::move(mock_uart));
    std::this_thread::sleep_for(std::chrono::milliseconds(startup_interval_ms+tick_interval_ms+1));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(garbage_ret_val));
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms+1));

    EXPECT_FALSE(estopManager.isEstopStatePlay());
}

