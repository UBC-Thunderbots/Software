#include "software/world/robot_state_with_timestamp.h"

#include <gtest/gtest.h>

TEST(RobotStateWithTimestampTest, get_position) {
    RobotStateWithTimestamp state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.position(), Point(1.1, -0.5));
}

TEST(RobotStateWithTimestampTest, get_velocity) {
    RobotStateWithTimestamp state(RobotState(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half()), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.velocity(), Vector(3, 0));
}

TEST(RobotStateWithTimestampTest, get_orientation) {
    RobotStateWithTimestamp state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.orientation(), Angle::quarter());
}

TEST(RobotStateWithTimestampTest, get_angular_velocity) {
    RobotStateWithTimestamp state(RobotState(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half()), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.angularVelocity(), AngularVelocity::half());
}

TEST(RobotStateWithTimestampTest, compare_identical_states) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_position) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.6), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_velocity) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.5), Vector(3, 1.2), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_orientation) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.5), Vector(3, 0), Angle::half(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_angular_velocity) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::zero(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_timestamps) {
    RobotStateWithTimestamp state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    RobotStateWithTimestamp state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(), AngularVelocity::half(), Timestamp::fromSeconds(0.2));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}
