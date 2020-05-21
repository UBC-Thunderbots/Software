#include "software/world/timestamped_robot_state.h"

#include <gtest/gtest.h>

TEST(RobotStateWithTimestampTest, get_timestamp)
{
    TimestampedRobotState state(RobotState(Point(1.1, -0.5), Vector(3, 0),
                                           Angle::quarter(), AngularVelocity::half()),
                                Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.timestamp(), Timestamp::fromSeconds(0.1));
}

TEST(RobotStateWithTimestampTest, get_robot_state)
{
    TimestampedRobotState state(RobotState(Point(1.1, -0.5), Vector(3, 0),
                                           Angle::quarter(), AngularVelocity::half()),
                                Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.robotState(), RobotState(Point(1.1, -0.5), Vector(3, 0),
                                             Angle::quarter(), AngularVelocity::half()));
}

TEST(RobotStateWithTimestampTest, compare_identical_states)
{
    TimestampedRobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_position)
{
    TimestampedRobotState state_1(Point(1.1, -0.6), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_velocity)
{
    TimestampedRobotState state_1(Point(1.1, -0.5), Vector(3, 1.2), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_orientation)
{
    TimestampedRobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::half(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_angular_velocity)
{
    TimestampedRobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::zero(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateWithTimestampTest, compare_states_with_different_timestamps)
{
    TimestampedRobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    TimestampedRobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half(), Timestamp::fromSeconds(0.2));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}
