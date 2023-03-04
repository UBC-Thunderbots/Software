#include "software/jetson_nano/services/network/proto_tracker.h"

#include <gtest/gtest.h>

TEST(ProtoTrackerTest, no_loss_rate)
{
    ProtoTracker tracker = ProtoTracker("");
    for (int i = 0; i <= 5; i++)
    {
        tracker.send(i);
        EXPECT_EQ(0, tracker.getLossRate());
    }
}

TEST(ProtoTrackerTest, half_loss_rate)
{
    ProtoTracker tracker = ProtoTracker("");
    tracker.send(0);
    EXPECT_EQ(0, tracker.getLossRate());
    tracker.send(2);
    EXPECT_NEAR(1.0 / 3, tracker.getLossRate(), 1e-4);
    tracker.send(4);
    EXPECT_NEAR(2.0 / 5, tracker.getLossRate(), 1e-4);
    tracker.send(6);
    EXPECT_NEAR(3.0 / 7, tracker.getLossRate(), 1e-4);
}

TEST(ProtoTrackerTest, two_loss_proto)
{
    ProtoTracker tracker = ProtoTracker("");
    tracker.send(0);
    EXPECT_EQ(0, tracker.getLossRate());
    tracker.send(2);
    EXPECT_NEAR(1.0 / 3, tracker.getLossRate(), 1e-4);
    tracker.send(3);
    EXPECT_NEAR(1.0 / 4, tracker.getLossRate(), 1e-4);
    tracker.send(4);
    EXPECT_NEAR(1.0 / 5, tracker.getLossRate(), 1e-4);
    tracker.send(6);
    EXPECT_NEAR(2.0 / 7, tracker.getLossRate(), 1e-4);
}

TEST(ProtoTrackerTest, ai_reset_test)
{
    ProtoTracker tracker = ProtoTracker("");
    tracker.send(0);
    EXPECT_EQ(0, tracker.getLossRate());
    tracker.send(98);
    EXPECT_NEAR(97.0 / 99, tracker.getLossRate(), 1e-4);
    tracker.send(99);
    EXPECT_NEAR(97.0 / 100, tracker.getLossRate(), 1e-4);
    tracker.send(100);
    EXPECT_NEAR(97.0 / 100, tracker.getLossRate(), 1e-4);
    tracker.send(101);
    EXPECT_NEAR(96.0 / 100, tracker.getLossRate(), 1e-4);
    tracker.send(0);  // AI reset
    EXPECT_EQ(0, tracker.getLossRate());
}

TEST(ProtoTrackerTest, out_of_order_test)
{
    ProtoTracker tracker = ProtoTracker("");
    tracker.send(0);
    EXPECT_EQ(0, tracker.getLossRate());
    tracker.send(1);
    EXPECT_EQ(0, tracker.getLossRate());
    tracker.send(2);
    EXPECT_EQ(0, tracker.getLossRate());
    EXPECT_TRUE(tracker.isLastValid());
    tracker.send(1);  // out of order
    EXPECT_EQ(0, tracker.getLossRate());
    EXPECT_FALSE(tracker.isLastValid());
}
