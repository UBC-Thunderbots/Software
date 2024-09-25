#include "software/test_util/fake_clock.h"

#include <gtest/gtest.h>

TEST(FakeClockTest, time_point_now_should_not_change_without_advancing)
{
    FakeClock::reset();
    FakeClock::time_point t0 = FakeClock::now();
    FakeClock::time_point t1 = FakeClock::now();
    EXPECT_EQ(std::chrono::milliseconds(0), t1 - t0);
}

TEST(FakeClockTest, time_point_now_should_move_forward_after_advancing)
{
    FakeClock::reset();
    FakeClock::time_point t0 = FakeClock::now();

    FakeClock::advance(std::chrono::milliseconds(100));
    FakeClock::time_point t1 = FakeClock::now();
    EXPECT_EQ(std::chrono::milliseconds(100), t1 - t0);

    FakeClock::advance(std::chrono::milliseconds(50));
    FakeClock::time_point t2 = FakeClock::now();
    EXPECT_EQ(std::chrono::milliseconds(150), t2 - t0);
}

TEST(FakeClockTest, reset_should_set_time_point_now_to_clock_epoch)
{
    FakeClock::reset();
    FakeClock::time_point t0 = FakeClock::now();

    FakeClock::advance(std::chrono::milliseconds(1000));
    FakeClock::time_point t1 = FakeClock::now();
    EXPECT_EQ(std::chrono::milliseconds(1000), t1 - t0);

    FakeClock::reset();
    FakeClock::time_point t2 = FakeClock::now();
    EXPECT_EQ(std::chrono::milliseconds(0), t2 - t0);
}
