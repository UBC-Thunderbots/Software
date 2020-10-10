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
    EXPECT_EQ(current_time, robot.timestamp());
}

TEST_F(RobotTest, construct_with_initial_state)
{
    Robot robot = Robot(3,
                        RobotState(Point(1, 1), Vector(-0.3, 0), Angle::fromRadians(2.2),
                                   AngularVelocity::fromRadians(-0.6)),
                        current_time);

    EXPECT_EQ(3, robot.id());
    EXPECT_EQ(Point(1, 1), robot.position());
    EXPECT_EQ(Vector(-0.3, 0), robot.velocity());
    EXPECT_EQ(Angle::fromRadians(2.2), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(-0.6), robot.angularVelocity());
    EXPECT_EQ(current_time, robot.timestamp());
}

TEST_F(RobotTest, update_state_with_all_params)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    robot.updateState(RobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                                 AngularVelocity::fromRadians(1.1)),
                      half_second_future);

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(1.1), robot.angularVelocity());
    EXPECT_EQ(half_second_future, robot.timestamp());
}

TEST_F(RobotTest, update_state_with_new_robot)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    RobotState update_robot(Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                            robot.angularVelocity());

    robot.updateState(update_robot, current_time);

    EXPECT_EQ(robot.currentState(), update_robot);
    EXPECT_EQ(robot.timestamp(), current_time);
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

TEST(RobotIsNearDribblerTest, ball_near_dribbler_directly_in_front_of_robot)
{
    Point ball_position = Point(0.07, 0);
    Timestamp timestamp = Timestamp::fromSeconds(0);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_TRUE(robot.isNearDribbler(ball_position));
}

TEST(RobotIsNearDribblerTest, ball_near_dribbler_ball_to_side_of_robot)
{
    Point ball_position = Point(0.07, 0);
    Timestamp timestamp = Timestamp::fromSeconds(0);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::half(), AngularVelocity::zero(),
                        timestamp);
    EXPECT_FALSE(robot.isNearDribbler(ball_position));
}

TEST(RobotIsNearDribblerTest, ball_near_dribbler_robot_moving_ball_in_dribbler)
{
    Point ball_position = Point(0.07, 0);
    Timestamp timestamp = Timestamp::fromSeconds(1);

    Robot robot = Robot(0, Point(0, 0), Vector(1, 1), Angle::zero(),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(robot.isNearDribbler(ball_position));
}

TEST(RobotIsNearDribblerTest, ball_near_dribbler_ball_far_away_from_robot)
{
    Point ball_position = Point(-1, -2);
    Timestamp timestamp = Timestamp::fromSeconds(0);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_FALSE(robot.isNearDribbler(ball_position));
}

TEST(RobotIsNearDribblerTest, ball_near_dribbler_robot_on_angle_with_ball_in_dribbler)
{
    Point ball_position = Point(0.035, 0.06);
    Timestamp timestamp = Timestamp::fromSeconds(0);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::fromDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(robot.isNearDribbler(ball_position));
}

TEST_F(RobotTest, get_unavailable_capabilities)
{
    std::set<RobotCapability> unavailable_capabilities = {
        RobotCapability::Dribble,
        RobotCapability::Chip,
    };

    Robot robot =
        Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
              AngularVelocity::fromDegrees(25), current_time, unavailable_capabilities);

    EXPECT_EQ(unavailable_capabilities, robot.getUnavailableCapabilities());
}

TEST_F(RobotTest, get_available_capabilities)
{
    std::set<RobotCapability> unavailable_capabilities = {
        RobotCapability::Dribble,
        RobotCapability::Chip,
    };

    Robot robot =
        Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
              AngularVelocity::fromDegrees(25), current_time, unavailable_capabilities);

    // available capabilities = all capabilities - unavailable capabilities
    std::set<RobotCapability> all_capabilities = allRobotCapabilities();
    std::set<RobotCapability> expected_capabilities;
    std::set_difference(
        all_capabilities.begin(), all_capabilities.end(),
        unavailable_capabilities.begin(), unavailable_capabilities.end(),
        std::inserter(expected_capabilities, expected_capabilities.begin()));

    EXPECT_EQ(expected_capabilities, robot.getAvailableCapabilities());
}
