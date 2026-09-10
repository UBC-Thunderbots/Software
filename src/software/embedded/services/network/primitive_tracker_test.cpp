#include "software/embedded/services/network/primitive_tracker.h"

#include <gtest/gtest.h>

#include <chrono>

namespace
{
TbotsProto::Primitive makePrimitive(uint64_t seq_num)
{
    TbotsProto::Primitive primitive;
    primitive.set_sequence_number(seq_num);
    return primitive;
}

double currentEpochTimeInSeconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
               .count() /
           1000000.0;
}
}  // namespace

TEST(PrimitiveTrackerTest, no_loss_rate)
{
    PrimitiveTracker tracker;
    for (int i = 0; i <= 5; i++)
    {
        tracker.track(makePrimitive(i));
        EXPECT_EQ(0, tracker.getPrimitiveLossRate());
    }
}

TEST(PrimitiveTrackerTest, half_loss_rate)
{
    PrimitiveTracker tracker;
    tracker.track(makePrimitive(0));
    EXPECT_EQ(0, tracker.getPrimitiveLossRate());
    tracker.track(makePrimitive(2));
    EXPECT_NEAR(1.0 / 3, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(4));
    EXPECT_NEAR(2.0 / 5, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(6));
    EXPECT_NEAR(3.0 / 7, tracker.getPrimitiveLossRate(), 1e-4);
}

TEST(PrimitiveTrackerTest, two_loss_proto)
{
    PrimitiveTracker tracker;
    tracker.track(makePrimitive(0));
    EXPECT_EQ(0, tracker.getPrimitiveLossRate());
    tracker.track(makePrimitive(2));
    EXPECT_NEAR(1.0 / 3, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(3));
    EXPECT_NEAR(1.0 / 4, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(4));
    EXPECT_NEAR(1.0 / 5, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(6));
    EXPECT_NEAR(2.0 / 7, tracker.getPrimitiveLossRate(), 1e-4);
}

TEST(PrimitiveTrackerTest, ai_reset_test)
{
    PrimitiveTracker tracker;
    tracker.track(makePrimitive(0));
    EXPECT_EQ(0, tracker.getPrimitiveLossRate());
    tracker.track(makePrimitive(98));
    EXPECT_NEAR(97.0 / 99, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(99));
    EXPECT_NEAR(97.0 / 100, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(100));
    EXPECT_NEAR(97.0 / 100, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(101));
    EXPECT_NEAR(96.0 / 100, tracker.getPrimitiveLossRate(), 1e-4);
    tracker.track(makePrimitive(0));  // AI reset
    EXPECT_EQ(0, tracker.getPrimitiveLossRate());
}

TEST(PrimitiveTrackerTest, out_of_order_test)
{
    PrimitiveTracker tracker;
    tracker.track(makePrimitive(0));
    tracker.track(makePrimitive(1));
    tracker.track(makePrimitive(2));
    EXPECT_EQ(0, tracker.getPrimitiveLossRate());

    tracker.track(makePrimitive(1));  // out of order, ignored

    EXPECT_EQ(0, tracker.getPrimitiveLossRate());
    const auto latest = tracker.getLatestPrimitive();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(2, latest->sequence_number());
}

TEST(PrimitiveTrackerTest, get_latest_primitive_returns_once)
{
    PrimitiveTracker tracker;

    tracker.track(makePrimitive(1));
    auto latest = tracker.getLatestPrimitive();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(1, latest->sequence_number());

    // The primitive has been consumed, so the next call returns nullopt
    EXPECT_FALSE(tracker.getLatestPrimitive().has_value());

    tracker.track(makePrimitive(2));
    latest = tracker.getLatestPrimitive();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(2, latest->sequence_number());
}

TEST(PrimitiveTrackerTest, update_primitive_log_sets_adjusted_time_sent)
{
    PrimitiveTracker tracker;

    const double epoch_now          = currentEpochTimeInSeconds();
    TbotsProto::Primitive primitive = makePrimitive(7);
    primitive.mutable_time_sent()->set_epoch_timestamp_seconds(epoch_now - 0.05);
    tracker.track(primitive);

    TbotsProto::RobotStatus robot_status;
    robot_status.set_last_handled_primitive_seq_num(7);
    tracker.updatePrimitiveLog(robot_status);

    ASSERT_TRUE(robot_status.has_adjusted_time_sent());
    // adjusted_time_sent = thunderscope_sent_time + processing_time, which is roughly
    // the sent time since processing_time is near zero.
    EXPECT_NEAR(epoch_now - 0.05,
                robot_status.adjusted_time_sent().epoch_timestamp_seconds(), 0.05);
}
