#include "software/world/robot.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

class RobotTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        // An arbitrary fixed point in time
        // We use this fixed point in time to make the tests deterministic.
        current_time       = Timestamp::fromSeconds(123);
        half_second_future = current_time + Duration::fromMilliseconds(500);
        one_second_future  = current_time + Duration::fromSeconds(1);
        one_second_past    = current_time - Duration::fromSeconds(1);
    }

    Timestamp current_time;
    Timestamp half_second_future;
    Timestamp one_second_future;
    Timestamp one_second_past;
};

TEST_F(RobotTest, construct_with_all_params)
{
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);

    EXPECT_EQ(3, robot.id());
    EXPECT_EQ(Point(1, 1), robot.position());
    EXPECT_EQ(Vector(-0.3, 0), robot.velocity());
    EXPECT_EQ(Angle::fromRadians(2.2), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(-0.6), robot.angularVelocity());
    EXPECT_EQ(current_time, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, construct_with_initial_state)
{
    Robot robot = Robot(
        3, TimestampedRobotState(Point(1, 1), Vector(-0.3, 0), Angle::fromRadians(2.2),
                                 AngularVelocity::fromRadians(-0.6), current_time));

    EXPECT_EQ(3, robot.id());
    EXPECT_EQ(Point(1, 1), robot.position());
    EXPECT_EQ(Vector(-0.3, 0), robot.velocity());
    EXPECT_EQ(Angle::fromRadians(2.2), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(-0.6), robot.angularVelocity());
    EXPECT_EQ(current_time, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_with_all_params)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(1.1), robot.angularVelocity());
    EXPECT_EQ(half_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_with_new_robot)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    TimestampedRobotState update_robot =
        TimestampedRobotState(Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                              robot.angularVelocity(), current_time);

    robot.updateState(update_robot);

    EXPECT_EQ(robot.currentState(), update_robot);
}

TEST_F(RobotTest, get_position_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Point(-1.2, 3), robot.position());
}


TEST_F(RobotTest, get_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Vector(-0.5, 2.6), robot.velocity());
}

TEST_F(RobotTest, get_orientation_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Angle::quarter(), robot.orientation());
}

TEST_F(RobotTest, get_angular_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(AngularVelocity::fromRadians(0.7), robot.angularVelocity());
}

TEST_F(RobotTest, equality_operator_compare_same_robot)
{
    Robot robot = Robot(0, Point(1, -1.5), Vector(-0.7, -0.55), Angle::fromDegrees(100),
                        AngularVelocity::fromDegrees(30), current_time);

    EXPECT_EQ(robot, robot);
}

TEST_F(RobotTest, equality_operator_robots_with_different_id)
{
    Robot robot_0 = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                          AngularVelocity::fromDegrees(25), current_time);

    Robot robot_1 = Robot(1, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                          AngularVelocity::fromDegrees(25), current_time);

    EXPECT_NE(robot_0, robot_1);
}

TEST_F(RobotTest, equality_operator_robots_with_different_position)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(-3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                              AngularVelocity::fromDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_velocity)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(), Angle::fromDegrees(0),
                              AngularVelocity::fromDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_orientation)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(45),
                              AngularVelocity::fromDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_angular_velocity)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                              AngularVelocity::fromDegrees(-70), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_timestamp)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                              AngularVelocity::fromDegrees(25), one_second_future);

    EXPECT_EQ(robot, robot_other);
}

TEST_F(RobotTest, get_position_history)
{
    std::vector prevPositions = {Point(-1.3, 3), Point(-1.2, 3), Point(3, 1.2)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));

    RobotHistory previous_states = robot.getPreviousStates();
    std::vector<Point> previous_positions{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_positions.push_back(previous_states.at(i).state().position());
    }
    EXPECT_EQ(prevPositions, previous_positions);
}

TEST_F(RobotTest, get_velocity_history)
{
    std::vector prevVelocities = {Vector(2.3, -0.05), Vector(2.2, -0.05), Vector(-3, 1)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));

    RobotHistory previous_states = robot.getPreviousStates();
    std::vector<Vector> previous_velocities{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_velocities.push_back(previous_states.at(i).state().velocity());
    }
    EXPECT_EQ(prevVelocities, previous_velocities);
}

TEST_F(RobotTest, get_orientation_history)
{
    std::vector prevOrientations = {Angle::quarter(), Angle::quarter(),
                                    Angle::fromDegrees(0)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));

    RobotHistory previous_states = robot.getPreviousStates();
    std::vector<Angle> previous_orientations{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_orientations.push_back(previous_states.at(i).state().orientation());
    }
    EXPECT_EQ(prevOrientations, previous_orientations);
}

TEST_F(RobotTest, get_angular_velocity_history)
{
    std::vector prevAngularVelocities = {
        AngularVelocity::fromRadians(1.2),
        AngularVelocity::fromRadians(1.1),
        AngularVelocity::fromDegrees(25),
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.2), half_second_future));

    RobotHistory previous_states = robot.getPreviousStates();
    std::vector<AngularVelocity> previous_angular_velocities{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_angular_velocities.push_back(
            previous_states.at(i).state().angularVelocity());
    }
    EXPECT_EQ(prevAngularVelocities, previous_angular_velocities);
}

TEST_F(RobotTest, get_timestamp_history)
{
    std::vector prevTimestamps = {half_second_future, half_second_future, current_time};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.2), half_second_future));

    RobotHistory previous_states = robot.getPreviousStates();
    std::vector<Timestamp> previous_timestamps{};
    for (size_t i = 0; i < previous_states.size(); i++)
    {
        previous_timestamps.push_back(previous_states.at(i).timestamp());
    }
    EXPECT_EQ(prevTimestamps, previous_timestamps);
}

TEST_F(RobotTest, get_capabilities_blacklist)
{
    std::set<RobotCapability> blacklist = {
        RobotCapability::Dribble,
        RobotCapability::Chip,
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3, blacklist);

    EXPECT_EQ(blacklist, robot.getCapabilitiesBlacklist());
}

TEST_F(RobotTest, get_capabilities_whitelist)
{
    std::set<RobotCapability> blacklist = {
        RobotCapability::Dribble,
        RobotCapability::Chip,
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3, blacklist);

    // whitelist = all capabilities - blacklist
    std::set<RobotCapability> all_capabilities = allRobotCapabilities();
    std::set<RobotCapability> expected_whitelist;
    std::set_difference(all_capabilities.begin(), all_capabilities.end(),
                        blacklist.begin(), blacklist.end(),
                        std::inserter(expected_whitelist, expected_whitelist.begin()));

    EXPECT_EQ(expected_whitelist, robot.getCapabilitiesWhitelist());
}
