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
        EXPECT_TRUE(tracker.track(makePrimitive(i)));
        EXPECT_EQ(0, tracker.getPacketLoss());
    }
}

TEST(PrimitiveTrackerTest, half_loss_rate)
{
    PrimitiveTracker tracker;
    EXPECT_TRUE(tracker.track(makePrimitive(0)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_TRUE(tracker.track(makePrimitive(2)));
    EXPECT_NEAR(1.0 / 3, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(4)));
    EXPECT_NEAR(2.0 / 5, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(6)));
    EXPECT_NEAR(3.0 / 7, tracker.getPacketLoss(), 1e-4);
}

TEST(PrimitiveTrackerTest, two_loss_proto)
{
    PrimitiveTracker tracker;
    EXPECT_TRUE(tracker.track(makePrimitive(0)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_TRUE(tracker.track(makePrimitive(2)));
    EXPECT_NEAR(1.0 / 3, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(3)));
    EXPECT_NEAR(1.0 / 4, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(4)));
    EXPECT_NEAR(1.0 / 5, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(6)));
    EXPECT_NEAR(2.0 / 7, tracker.getPacketLoss(), 1e-4);
}

TEST(PrimitiveTrackerTest, ai_reset_test)
{
    PrimitiveTracker tracker;
    EXPECT_TRUE(tracker.track(makePrimitive(0)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_TRUE(tracker.track(makePrimitive(98)));
    EXPECT_NEAR(97.0 / 99, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(99)));
    EXPECT_NEAR(97.0 / 100, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(100)));
    EXPECT_NEAR(97.0 / 100, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(101)));
    EXPECT_NEAR(96.0 / 100, tracker.getPacketLoss(), 1e-4);
    EXPECT_TRUE(tracker.track(makePrimitive(0)));  // AI reset
    EXPECT_EQ(0, tracker.getPacketLoss());
}

TEST(PrimitiveTrackerTest, out_of_order_test)
{
    PrimitiveTracker tracker;
    EXPECT_TRUE(tracker.track(makePrimitive(0)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_TRUE(tracker.track(makePrimitive(1)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_TRUE(tracker.track(makePrimitive(2)));
    EXPECT_EQ(0, tracker.getPacketLoss());
    EXPECT_FALSE(tracker.track(makePrimitive(1)));  // out of order
    EXPECT_EQ(0, tracker.getPacketLoss());
}

TEST(PrimitiveTrackerTest, average_round_trip_time)
{
    PrimitiveTracker tracker;
    EXPECT_EQ(0.0, tracker.getAverageRoundTripTime());

    const double epoch_now = currentEpochTimeInSeconds();
    for (int i = 0; i < 3; i++)
    {
        TbotsProto::Primitive primitive = makePrimitive(i);
        primitive.mutable_time_sent()->set_epoch_timestamp_seconds(epoch_now - 0.1);
        tracker.track(primitive);
    }

    // Each primitive was sent 0.1s before it was received, so the average round-trip
    // time should be approximately 0.1s.
    EXPECT_NEAR(0.1, tracker.getAverageRoundTripTime(), 0.05);
}

TEST(PrimitiveTrackerTest, last_primitive_received_time_ignores_out_of_order)
{
    PrimitiveTracker tracker;

    EXPECT_TRUE(tracker.track(makePrimitive(5)));
    const auto last_valid_time = tracker.getLastPrimitiveReceivedTime();

    // Out-of-order primitive should not update the last-received time
    EXPECT_FALSE(tracker.track(makePrimitive(3)));
    EXPECT_EQ(last_valid_time, tracker.getLastPrimitiveReceivedTime());

    EXPECT_TRUE(tracker.track(makePrimitive(6)));
    EXPECT_GE(tracker.getLastPrimitiveReceivedTime(), last_valid_time);
}

TEST(PrimitiveTrackerTest, update_primitive_log_sets_adjusted_time_sent)
{
    PrimitiveTracker tracker;

    const double epoch_now          = currentEpochTimeInSeconds();
    TbotsProto::Primitive primitive = makePrimitive(7);
    primitive.mutable_time_sent()->set_epoch_timestamp_seconds(epoch_now - 0.05);
    tracker.track(primitive);

    TbotsProto::RobotStatus robot_status;
    robot_status.set_last_handled_primitive_set(7);
    tracker.updatePrimitiveLog(robot_status);

    ASSERT_TRUE(robot_status.has_adjusted_time_sent());
    // adjusted_time_sent = thunderscope_sent_time + processing_time, which is roughly
    // the sent time since processing_time is near zero.
    EXPECT_NEAR(epoch_now - 0.05,
                robot_status.adjusted_time_sent().epoch_timestamp_seconds(), 0.05);
}
