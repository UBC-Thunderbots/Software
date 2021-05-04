#include "threaded_estop_reader.h"

#include <gmock/gmock.h>

#include "gtest/gtest.h"
#include "mock_uart_communication.h"
#include "uart_communication.h"

static constexpr unsigned char STOP = 0;
static constexpr unsigned char PLAY = 1;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;
using ::testing::TimeInMillis;

TEST(ThreadedEstopReaderTest, estop_play_is_false_by_default_before_startup)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> ret_val(1, PLAY);

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, serialRead(_)).WillRepeatedly(Return(ret_val));
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));


    int startup_interval_ms = 500;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms);

    EXPECT_FALSE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest, estop_tick_is_called_multiple_times)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    int test_timeout_ms                 = 100;
    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    auto mock_uart_ptr = mock_uart.get();

    int startup_interval_ms = 5;
    std::mutex m;
    std::unique_lock lock(m);
    std::condition_variable cv;
    bool ready;

    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .Times(AtLeast(5))
        .WillOnce(Return(play_ret_val))
        .WillOnce(Return(play_ret_val))
        .WillOnce(Return(play_ret_val))
        .WillOnce(Return(play_ret_val))
        .WillOnce(DoAll(InvokeWithoutArgs([&cv, &ready] {
                            ready = true;
                            cv.notify_one();
                        }),
                        Return(play_ret_val)))
        .WillRepeatedly(Return(play_ret_val));

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms);
    auto now = std::chrono::system_clock::now();
    EXPECT_TRUE(cv.wait_until(lock, now + std::chrono::milliseconds(test_timeout_ms),
                              [&ready] { return ready; }));
}

TEST(ThreadedEstopReaderTest, estop_state_changes_based_on_read_val)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();

    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> stop_ret_val(1, STOP);

    int test_timeout_ms     = 1000;
    int startup_interval_ms = 5;
    auto mock_uart_ptr      = mock_uart.get();

    std::mutex m;
    std::unique_lock lock(m);
    std::condition_variable cv;
    std::atomic_bool ready;


    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .WillOnce(Return(play_ret_val))
        .WillOnce(DoAll(InvokeWithoutArgs([&cv, &ready] {
                            ready = true;
                            cv.notify_one();
                        }),
                        Return(play_ret_val)))
        .WillRepeatedly(Return(play_ret_val));

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms);

    EXPECT_TRUE(cv.wait_until(
        lock,
        std::chrono::system_clock::now() +
            std::chrono::milliseconds(startup_interval_ms + test_timeout_ms),
        [&ready] { return ready == true; }));
    EXPECT_TRUE(estopReader.isEstopPlay());

    // change uart return value to stop
    ready = false;
    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .WillOnce(Return(stop_ret_val))
        .WillOnce(DoAll(InvokeWithoutArgs([&cv, &ready] {
                            ready = true;
                            cv.notify_one();
                        }),
                        Return(stop_ret_val)))
        .WillRepeatedly(Return(stop_ret_val));
    EXPECT_TRUE(cv.wait_until(
        lock,
        std::chrono::system_clock::now() + std::chrono::milliseconds(test_timeout_ms),
        [&ready] { return ready == true; }));
    EXPECT_FALSE(estopReader.isEstopPlay());

    // change uart return value back to play
    ready = false;
    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .WillOnce(Return(play_ret_val))
        .WillOnce(DoAll(InvokeWithoutArgs([&cv, &ready] {
                            ready = true;
                            cv.notify_one();
                        }),
                        Return(play_ret_val)))
        .WillRepeatedly(Return(play_ret_val));
    EXPECT_TRUE(cv.wait_until(
        lock,
        std::chrono::system_clock::now() + std::chrono::milliseconds(test_timeout_ms),
        [&ready] { return ready == true; }));
    EXPECT_TRUE(estopReader.isEstopPlay());
}

TEST(ThreadedEstopReaderTest, estop_play_is_false_after_reading_unexpected_message)
{
    std::unique_ptr<MockUart> mock_uart = std::make_unique<MockUart>();
    auto mock_uart_ptr                  = mock_uart.get();

    int test_timeout_ms = 1000;

    unsigned char arbitrary_garbage_val = 0b10111011;
    std::vector<unsigned char> play_ret_val(1, PLAY);
    std::vector<unsigned char> garbage_ret_val(1, arbitrary_garbage_val);

    std::mutex m;
    std::unique_lock lock(m);
    std::condition_variable cv;
    std::atomic_bool ready;

    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .Times(AtLeast(3))
        .WillOnce(Return(play_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(DoAll(InvokeWithoutArgs([&cv, &ready] {
                            ready = true;
                            cv.notify_one();
                        }),
                        Return(garbage_ret_val)))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillRepeatedly(Return(play_ret_val));

    int startup_interval_ms = 1;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms);

    EXPECT_TRUE(cv.wait_until(
        lock,
        std::chrono::system_clock::now() +
            std::chrono::milliseconds(startup_interval_ms + test_timeout_ms),
        [&ready] { return ready == true; }));
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
    int test_timeout_ms = 1000;


    std::mutex m;
    std::unique_lock lock(m);
    std::condition_variable cv;
    std::atomic_bool ready;

    auto mock_uart_ptr = mock_uart.get();
    EXPECT_CALL(*mock_uart_ptr, flushSerialPort(_)).WillRepeatedly(Return(true));

    EXPECT_CALL(*mock_uart_ptr, serialRead(_))
        .Times(AtLeast(5))
        .WillOnce(Return(play_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(garbage_ret_val))
        .WillOnce(Return(play_ret_val))
        .WillRepeatedly(DoAll(InvokeWithoutArgs([&cv, &ready] {
                                  ready = true;
                                  cv.notify_one();
                              }),
                              Return(play_ret_val)));

    int startup_interval_ms = 5;

    ThreadedEstopReader estopReader(std::move(mock_uart), startup_interval_ms);

    EXPECT_TRUE(cv.wait_until(
        lock,
        std::chrono::system_clock::now() +
            std::chrono::milliseconds(startup_interval_ms + test_timeout_ms),
        [&ready] { return ready == true; }));
    EXPECT_TRUE(estopReader.isEstopPlay());
}
