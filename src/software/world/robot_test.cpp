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

TEST_F(RobotTest, update_state_to_predicted_state_with_future_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                        AngularVelocity::fromRadians(2), current_time);

    robot.updateStateToPredictedState(one_second_future);

    EXPECT_EQ(Point(4.5, -1), robot.position());
    EXPECT_EQ(Vector(3.5, 1), robot.velocity());
    EXPECT_EQ(Angle::fromRadians(-0.3) + Angle::fromRadians(2), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(2), robot.angularVelocity());
    EXPECT_EQ(one_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_to_predicted_state_with_positive_duration)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                        AngularVelocity::fromRadians(2), current_time);

    robot.updateStateToPredictedState(Duration::fromSeconds(1));

    EXPECT_EQ(Point(4.5, -1), robot.position());
    EXPECT_EQ(Vector(3.5, 1), robot.velocity());
    EXPECT_EQ(Angle::fromRadians(-0.3) + Angle::fromRadians(2), robot.orientation());
    EXPECT_EQ(AngularVelocity::fromRadians(2), robot.angularVelocity());
    EXPECT_EQ(one_second_future, robot.lastUpdateTimestamp());
}

TEST_F(RobotTest, update_state_to_predicted_state_with_past_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                        AngularVelocity::fromRadians(2), current_time);

    ASSERT_THROW(robot.updateStateToPredictedState(one_second_past),
                 std::invalid_argument);
}

TEST_F(RobotTest, update_state_to_predicted_state_with_negative_duration)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                        AngularVelocity::fromRadians(2), current_time);

    ASSERT_THROW(robot.updateStateToPredictedState(Duration::fromSeconds(-1)),
                 std::invalid_argument);
}

TEST_F(RobotTest, get_position_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Point(-1.2, 3), robot.position());
}

TEST_F(RobotTest, get_position_at_future_time_with_negative_robot_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, -2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Point(-1.4, 1.96),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(-1.7, 0.4),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(-2.7, -4.8),
              robot.estimatePositionAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_position_at_future_time_with_positive_robot_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

    EXPECT_EQ(Point(2.4, -1.6),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Point(4.5, -1),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(11.5, 1),
              robot_other.estimatePositionAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_position_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

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
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Vector(-0.5, 2.6), robot.velocity());
}

TEST_F(RobotTest, get_velocity_at_future_time_with_negative_robot_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, -2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Vector(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Vector(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Vector(-0.5, -2.6),
              robot.estimateVelocityAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_velocity_at_future_time_with_positive_robot_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

    EXPECT_EQ(Vector(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Vector(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Vector(3.5, 1),
              robot_other.estimateVelocityAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_velocity_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

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
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Angle::quarter(), robot.orientation());
}

TEST_F(RobotTest, get_orientation_at_future_time_with_positive_robot_angular_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(Angle::quarter() + Angle::fromRadians(0.28),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(Angle::quarter() + Angle::fromRadians(0.7),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Angle::quarter() + Angle::fromRadians(2.1),
              robot.estimateOrientationAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_orientation_at_future_time_with_negative_robot_angular_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

    EXPECT_EQ(
        Angle::fromRadians(-0.3) + Angle::fromRadians(0.8),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(
        Angle::fromRadians(-0.3) + Angle::fromRadians(2),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(
        Angle::fromRadians(-0.3) + Angle::fromRadians(6),
        robot_other.estimateOrientationAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_orientation_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

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
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(AngularVelocity::fromRadians(0.7), robot.angularVelocity());
}

TEST_F(RobotTest,
       get_angular_velocity_at_future_time_with_positive_robot_angular_velocity)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::fromRadians(0.7), current_time);

    EXPECT_EQ(AngularVelocity::fromRadians(0.7),
              robot.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(
        AngularVelocity::fromRadians(0.7),
        robot.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(1000)));
    EXPECT_EQ(
        AngularVelocity::fromRadians(0.7),
        robot.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest,
       get_angular_velocity_at_future_time_with_negative_robot_angular_velocity)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

    EXPECT_EQ(
        AngularVelocity::fromRadians(2),
        robot_other.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(400)));
    EXPECT_EQ(AngularVelocity::fromRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(
                  Duration::fromMilliseconds(1000)));
    EXPECT_EQ(AngularVelocity::fromRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(
                  Duration::fromMilliseconds(3000)));
}

TEST_F(RobotTest, get_angular_velocity_at_past_time)
{
    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                              AngularVelocity::fromRadians(2), current_time);

    ASSERT_THROW(
        robot_other.estimateAngularVelocityAtFutureTime(Duration::fromMilliseconds(-100)),
        std::invalid_argument);
    ASSERT_THROW(robot_other.estimateAngularVelocityAtFutureTime(
                     Duration::fromMilliseconds(-1000)),
                 std::invalid_argument);
}

TEST_F(RobotTest, get_last_update_timestamp)
{
    Robot robot = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::fromRadians(-0.3),
                        AngularVelocity::fromRadians(2), current_time);

    EXPECT_EQ(current_time, robot.lastUpdateTimestamp());

    robot.updateStateToPredictedState(half_second_future);

    EXPECT_EQ(half_second_future, robot.lastUpdateTimestamp());
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

    boost::circular_buffer<TimestampedRobotState> previous_states =
        robot.getPreviousStates();
    std::vector<Point> previous_positions{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_positions.push_back(previous_states.at(i).robotState().position());
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

    boost::circular_buffer<TimestampedRobotState> previous_states =
        robot.getPreviousStates();
    std::vector<Vector> previous_velocities{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_velocities.push_back(previous_states.at(i).robotState().velocity());
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

    boost::circular_buffer<TimestampedRobotState> previous_states =
        robot.getPreviousStates();
    std::vector<Angle> previous_orientations{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_orientations.push_back(previous_states.at(i).robotState().orientation());
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

    boost::circular_buffer<TimestampedRobotState> previous_states =
        robot.getPreviousStates();
    std::vector<AngularVelocity> previous_angular_velocities{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_angular_velocities.push_back(
            previous_states.at(i).robotState().angularVelocity());
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

    boost::circular_buffer<TimestampedRobotState> previous_states =
        robot.getPreviousStates();
    std::vector<Timestamp> previous_timestamps{};
    for (int i = 0; i < previous_states.size(); i++)
    {
        previous_timestamps.push_back(previous_states.at(i).timestamp());
    }
    EXPECT_EQ(prevTimestamps, previous_timestamps);
}

TEST_F(RobotTest, get_timestamp_index_fetches_first_index)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.2), one_second_future));

    EXPECT_EQ(0, robot.getHistoryIndexFromTimestamp(one_second_future));
}

TEST_F(RobotTest, get_timestamp_index_fetches_last_index)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.2), one_second_future));

    EXPECT_EQ(2, robot.getHistoryIndexFromTimestamp(current_time));
}

TEST_F(RobotTest, get_timestamp_index_no_matching_timestamp)
{
    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3);
    robot.updateState(
        TimestampedRobotState(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.1), half_second_future));
    robot.updateState(
        TimestampedRobotState(Point(-1.3, 3), Vector(2.3, -0.05), Angle::quarter(),
                              AngularVelocity::fromRadians(1.2), one_second_future));

    Timestamp no_matching_time =
        half_second_future +
        Duration::fromMilliseconds(POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS + 1.0);

    EXPECT_EQ(std::nullopt, robot.getHistoryIndexFromTimestamp(no_matching_time));
}

TEST_F(RobotTest, get_capabilities_blacklist)
{
    std::set<RobotCapabilities::Capability> blacklist = {
        RobotCapabilities::Capability::Dribble,
        RobotCapabilities::Capability::Chip,
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3, blacklist);

    EXPECT_EQ(blacklist, robot.getCapabilitiesBlacklist());
}

TEST_F(RobotTest, get_capabilities_whitelist)
{
    std::set<RobotCapabilities::Capability> blacklist = {
        RobotCapabilities::Capability::Dribble,
        RobotCapabilities::Capability::Chip,
    };

    Robot robot = Robot(0, Point(3, 1.2), Vector(-3, 1), Angle::fromDegrees(0),
                        AngularVelocity::fromDegrees(25), current_time, 3, blacklist);

    // whitelist = all capabilities - blacklist
    std::set<RobotCapabilities::Capability> all_capabilities =
        RobotCapabilities::allCapabilities();
    std::set<RobotCapabilities::Capability> expected_whitelist;
    std::set_difference(all_capabilities.begin(), all_capabilities.end(),
                        blacklist.begin(), blacklist.end(),
                        std::inserter(expected_whitelist, expected_whitelist.begin()));

    EXPECT_EQ(expected_whitelist, robot.getCapabilitiesWhitelist());
}
