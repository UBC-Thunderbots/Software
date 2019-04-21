#include "ai/world/robot.h"

#include <gtest/gtest.h>

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
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);

    EXPECT_EQ(3, robot.id());
    EXPECT_EQ(Point(1, 1), robot.position());
    EXPECT_EQ(Vector(-0.3, 0), robot.velocity());
    EXPECT_EQ(Angle::ofRadians(2.2), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(-0.6), robot.angularVelocity());
    EXPECT_EQ(current_time, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_with_all_params)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(1.1), robot.angularVelocity());
    EXPECT_EQ(half_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_with_new_robot_with_same_id)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    Robot update_robot = Robot(0, Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                               robot.angularVelocity(), current_time);

    robot.updateState(update_robot);

    EXPECT_EQ(robot, update_robot);
}

TEST_F(RobotTest, update_state_with_new_robot_with_different_id)
{
    Robot robot =
        Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(), current_time);

    Robot update_robot = Robot(1, Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                               robot.angularVelocity(), current_time);

    ASSERT_THROW(robot.updateState(update_robot), std::invalid_argument);
}

TEST_F(RobotTest, update_state_to_predicted_state_with_future_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                        AngularVelocity::ofRadians(2), current_time);

    robot.updateStateToPredictedState(one_second_future);

    EXPECT_EQ(Point(4.5, -1), robot.position());
    EXPECT_EQ(Point(3.5, 1), robot.velocity());
    EXPECT_EQ(Angle::ofRadians(-0.3) + Angle::ofRadians(2), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(2), robot.angularVelocity());
    EXPECT_EQ(one_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_to_predicted_state_with_positive_duration)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                        AngularVelocity::ofRadians(2), current_time);

    robot.updateStateToPredictedState(Duration::fromSeconds(1));

    EXPECT_EQ(Point(4.5, -1), robot.position());
    EXPECT_EQ(Point(3.5, 1), robot.velocity());
    EXPECT_EQ(Angle::ofRadians(-0.3) + Angle::ofRadians(2), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(2), robot.angularVelocity());
    EXPECT_EQ(one_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_to_predicted_state_with_past_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                        AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(robot.updateStateToPredictedState(one_second_past),
                 std::invalid_argument);
}

TEST_F(RobotTest, update_state_to_predicted_state_with_negative_duration)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                        AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(robot.updateStateToPredictedState(Duration::fromSeconds(-1)),
                 std::invalid_argument);
}

TEST_F(RobotTest, get_position_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Point(-1.2, 3), robot.position());
}

TEST_F(RobotTest, get_position_at_future_time_with_negative_robot_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, -2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Point(-1.4, 1.96),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(-1.7, 0.4),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(-2.7, -4.8),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_position_at_future_time_with_positive_robot_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    EXPECT_EQ(Point(2.4, -1.6),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(4.5, -1),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(11.5, 1),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_position_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(
        (robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(-100))),
        std::invalid_argument);
    ASSERT_THROW(
        (robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(-1000))),
        std::invalid_argument);
}

TEST_F(RobotTest, get_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Vector(-0.5, 2.6), robot.velocity());
}

TEST_F(RobotTest, get_velocity_at_future_time_with_negative_robot_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, -2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Point(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_velocity_at_future_time_with_positive_robot_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    EXPECT_EQ(Point(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_velocity_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(
        (robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(-100))),
        std::invalid_argument);
    ASSERT_THROW(
        (robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(-1000))),
        std::invalid_argument);
}

TEST_F(RobotTest, get_orientation_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Angle::quarter(), robot.orientation());
}

TEST_F(RobotTest, get_orientation_at_future_time_with_positive_robot_angular_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(0.28),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(0.7),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(2.1),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_orientation_at_future_time_with_negative_robot_angular_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    EXPECT_EQ(
        Angle::ofRadians(-0.3) + Angle::ofRadians(0.8),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(
        Angle::ofRadians(-0.3) + Angle::ofRadians(2),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(
        Angle::ofRadians(-0.3) + Angle::ofRadians(6),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_orientation_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(-100)),
        std::invalid_argument);
    ASSERT_THROW(
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(-1000)),
        std::invalid_argument);
}

TEST_F(RobotTest, get_angular_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(AngularVelocity::ofRadians(0.7), robot.angularVelocity());
}

TEST_F(RobotTest,
       get_angular_velocity_at_future_time_with_positive_robot_angular_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7), current_time);

    EXPECT_EQ(AngularVelocity::ofRadians(0.7),
              robot.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(AngularVelocity::ofRadians(0.7), robot.estimateAngularVelocityAtFutureTime(
                                                   Duration::fromMilliseconds(1000)));
    EXPECT_EQ(AngularVelocity::ofRadians(0.7), robot.estimateAngularVelocityAtFutureTime(
                                                   Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest,
       get_angular_velocity_at_future_time_with_negative_robot_angular_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    EXPECT_EQ(
        AngularVelocity::ofRadians(2),
        robot_other.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(AngularVelocity::ofRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(
                  Duration::fromMilliseconds(1000)));
    EXPECT_EQ(AngularVelocity::ofRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(
                  Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_angular_velocity_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2), current_time);

    ASSERT_THROW(
        robot_other.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(-100)),
        std::invalid_argument);
    ASSERT_THROW(robot_other.estimateAngularVelocityAtFutureTime(
                     Duration::fromMilliseconds(-1000)),
                 std::invalid_argument);
}

TEST_F(RobotTest, get_last_update_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                        AngularVelocity::ofRadians(2), current_time);

    EXPECT_EQ(current_time, robot.lastUpdateTimestamp());

    robot.updateStateToPredictedState(half_second_future);

    EXPECT_EQ(half_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, equality_operator_compare_same_robot)
{
    Robot robot = Robot(0, Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                        AngularVelocity::ofDegrees(30), current_time);

    EXPECT_EQ(robot, robot);
}

TEST_F(RobotTest, equality_operator_robots_with_different_id)
{
    Robot robot_0 = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                          AngularVelocity::ofDegrees(25), current_time);

    Robot robot_1 = Robot(1, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                          AngularVelocity::ofDegrees(25), current_time);

    EXPECT_NE(robot_0, robot_1);
}

TEST_F(RobotTest, equality_operator_robots_with_different_position)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(-3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                              AngularVelocity::ofDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_velocity)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(), Angle::ofDegrees(0),
                              AngularVelocity::ofDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_orientation)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(45),
                              AngularVelocity::ofDegrees(25), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_angular_velocity)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                              AngularVelocity::ofDegrees(-70), current_time);

    EXPECT_NE(robot, robot_other);
}

TEST_F(RobotTest, equality_operator_robots_with_different_timestamp)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time);

    Robot robot_other = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                              AngularVelocity::ofDegrees(25), one_second_future);

    EXPECT_EQ(robot, robot_other);
}

TEST_F(RobotTest, get_position_history)
{
    std::vector prevPositions = {Point(-1.3, 3), Point(-1.2, 3), Point(3, 1.2)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time, 3);
    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);
    robot.updateState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);

    EXPECT_EQ(prevPositions, robot.getPreviousPositions());
}

TEST_F(RobotTest, get_velocity_history)
{
    std::vector prevVelocities = {Vector(2.3, -0.05), Vector(2.2, -0.05), Vector(-3, 1)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time, 3);
    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);
    robot.updateState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);

    EXPECT_EQ(prevVelocities, robot.getPreviousVelocities());
}

TEST_F(RobotTest, get_orientation_history)
{
    std::vector prevOrientations = {Angle::quarter(), Angle::quarter(),
                                    Angle::ofDegrees(0)};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time, 3);
    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);
    robot.updateState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);

    EXPECT_EQ(prevOrientations, robot.getPreviousOrientations());
}

TEST_F(RobotTest, get_angular_velocity_history)
{
    std::vector prevAngularVelocities = {
        AngularVelocity::ofRadians(1.2),
        AngularVelocity::ofRadians(1.1),
        AngularVelocity::ofDegrees(25),
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time, 3);
    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);
    robot.updateState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.2), half_second_future);

    EXPECT_EQ(prevAngularVelocities, robot.getPreviousAngularVelocities());
}

TEST_F(RobotTest, get_timestamp_history)
{
    std::vector prevAngularVelocities = {half_second_future, half_second_future,
                                         current_time};

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::ofDegrees(0),
                        AngularVelocity::ofDegrees(25), current_time, 3);
    robot.updateState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.1), half_second_future);
    robot.updateState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                      AngularVelocity::ofRadians(1.2), half_second_future);

    EXPECT_EQ(prevAngularVelocities, robot.getPreviousTimestamps());
}
