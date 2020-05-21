#include "software/world/timestamped_robot_state.h"

#include <gtest/gtest.h>

TEST(RobotStateWithTimestampTest, get_position)
{
    TimestampedRobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.position(), Point(1.1, -0.5));
}

TEST(RobotStateWithTimestampTest, get_velocity)
{
    TimestampedRobotState state(RobotState(Point(1.1, -0.5), Vector(3, 0),
                                           Angle::quarter(), AngularVelocity::half()),
                                Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.velocity(), Vector(3, 0));
}

TEST(RobotStateWithTimestampTest, get_orientation)
{
    TimestampedRobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                                AngularVelocity::half(), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.orientation(), Angle::quarter());
}

TEST(RobotStateWithTimestampTest, get_angular_velocity)
{
    TimestampedRobotState state(RobotState(Point(1.1, -0.5), Vector(3, 0),
                                           Angle::quarter(), AngularVelocity::half()),
                                Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.angularVelocity(), AngularVelocity::half());
}

TEST(RobotStateWithTimestampTest, get_robot_state)
{
    TimestampedRobotState state(RobotState(Point(1.1, -0.5), Vector(3, 0),
                                           Angle::quarter(), AngularVelocity::half()),
                                Timestamp::fromSeconds(0.1));
    EXPECT_EQ(state.getRobotState(),
              RobotState(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                         AngularVelocity::half()));
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
