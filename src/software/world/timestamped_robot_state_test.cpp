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

TEST(RobotStateWithTimestampTest, update_with_set_robot_state)
{
    TimestampedRobotState timestamped_robot_state_1(
        Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
        AngularVelocity::fromRadians(1.1), Timestamp::fromSeconds(123));
    TimestampedRobotState timestamped_robot_state_2(
        Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
        AngularVelocity::fromRadians(1.1), Timestamp::fromSeconds(123));
    EXPECT_EQ(timestamped_robot_state_1, timestamped_robot_state_2);

    timestamped_robot_state_2.robotState().setBallInMouth(true);
    timestamped_robot_state_2.robotState().setTimeSinceLastChip(
        Duration::fromMilliseconds(11));
    timestamped_robot_state_2.robotState().setTimeSinceLastKick(
        Duration::fromMilliseconds(5));
    EXPECT_FALSE(timestamped_robot_state_2.robotState().timeSinceLastChip());
    EXPECT_FALSE(timestamped_robot_state_2.robotState().timeSinceLastKick());
    EXPECT_EQ(timestamped_robot_state_1, timestamped_robot_state_2);

    RobotState updated_state = timestamped_robot_state_2.robotState();
    updated_state.setBallInMouth(true);
    updated_state.setTimeSinceLastChip(Duration::fromMilliseconds(11));
    updated_state.setTimeSinceLastKick(Duration::fromMilliseconds(5));
    timestamped_robot_state_2.setRobotState(updated_state);

    EXPECT_EQ(timestamped_robot_state_2.robotState().timeSinceLastChip(),
              Duration::fromMilliseconds(11));
    EXPECT_EQ(timestamped_robot_state_2.robotState().timeSinceLastKick(),
              Duration::fromMilliseconds(5));
    EXPECT_NE(timestamped_robot_state_1, timestamped_robot_state_2);
}
