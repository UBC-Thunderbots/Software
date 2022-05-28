#include "software/ai/evaluation/pass.h"

#include <gtest/gtest.h>

#include "software/test_util/equal_within_tolerance.h"

class PassingEvaluationTest : public ::testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
};

TEST_F(PassingEvaluationTest, getTimeToOrientationForRobot_robot_at_desired_angle)
{
    Angle target_angle = Angle::half();
    Robot robot(0, {1, 1}, Vector(0, 0), Angle::half(), AngularVelocity::fromDegrees(0),
                Timestamp::fromSeconds(0));

    EXPECT_EQ(
        Duration::fromSeconds(0),
        getTimeToOrientationForRobot(robot.orientation(), target_angle, 4 * M_PI, 10.0));
}

TEST_F(PassingEvaluationTest,
       getTimeToOrientationForRobot_robot_opposite_to_desired_angle)
{
    // Because we can't guarantee that the robot angular acceleration isn't going to
    // increase to the point where we can't reach the max angular speed within
    // a half rotation, this is just a more loose test that checks if we got there
    // within roughly the expected time. Any more strict test would have to
    // basically re-write the function, which would be a bit pointless

    Angle target_angle = Angle::zero();
    Robot robot(0, {1, 1}, Vector(0, 0), Angle::half(), AngularVelocity::fromDegrees(0),
                Timestamp::fromSeconds(0));

    // Figure out a lower bound on the time required, based on us being able to constantly
    // accelerate at the max acceleration
    // s = ut + 1/2 * at^2, u = 0, s = pi/2
    // t = sqrt(2*s/a)
    double min_time_to_rotate = 0.56;

    // For the upper bound, just choose a time that's much greater then we would expect
    double max_time_to_rotate = 4.0;

    Duration t =
        getTimeToOrientationForRobot(robot.orientation(), target_angle, 4 * M_PI, 10);
    EXPECT_LE(
        Duration::fromSeconds(min_time_to_rotate),
        getTimeToOrientationForRobot(robot.orientation(), target_angle, 4 * M_PI, 10));
    EXPECT_GE(
        Duration::fromSeconds(max_time_to_rotate),
        getTimeToOrientationForRobot(robot.orientation(), target_angle, 4 * M_PI, 10));
}

TEST_F(PassingEvaluationTest, getTimeToPositionForRobot_already_at_dest)
{
    Point dest(1, 1);
    EXPECT_EQ(
        Duration::fromSeconds(0),
        getTimeToPositionForRobot(dest, dest, robot_constants.robot_max_speed_m_per_s,
                                  robot_constants.robot_max_acceleration_m_per_s_2, 0.5));
}

TEST_F(PassingEvaluationTest, getTimeToPositionForRobot_reaches_max_velocity)
{
    // Check that the robot reaches the dest in the at the expected time when
    // it has enough time that it accelerates up to it's maximum velocity
    // We set the distance here such that it the robot *should* always have time to
    // get to the max velocity, unless our robots suddenly get *much* faster

    Point dest(1, 1);
    Point robot_location(40, 40);

    double distance_to_dest = (robot_location - dest).length();

    double acceleration_time = robot_constants.robot_max_speed_m_per_s /
                               robot_constants.robot_max_acceleration_m_per_s_2;
    // x = v*t + 1/2*a*t^2, v = initial velocity = 0
    double acceleration_distance = 0.5 *
                                   robot_constants.robot_max_acceleration_m_per_s_2 *
                                   std::pow(acceleration_time, 2);
    double time_at_max_vel = (distance_to_dest - 2 * acceleration_distance) /
                             robot_constants.robot_max_speed_m_per_s;

    double travel_time = 2 * acceleration_time + time_at_max_vel;

    EXPECT_EQ(Duration::fromSeconds(travel_time),
              getTimeToPositionForRobot(
                  robot_location, dest, robot_constants.robot_max_speed_m_per_s,
                  robot_constants.robot_max_acceleration_m_per_s_2));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_reaches_max_velocity_with_tolerance)
{
    // Check that the robot reaches the dest in the at the expected time when
    // it has enough time that it accelerates up to it's maximum velocity
    // We set the distance here such that it the robot *should* always have time to
    // get to the max velocity, unless our robots suddenly get *much* faster

    // The point we're trying to move to
    Point target_location(0, 1);

    // The first point within tolerance of the goal on the path of the robot
    Point first_point_in_tolerance(0, 1.5);

    Point robot_location(0, 40);

    double distance_to_dest = (robot_location - first_point_in_tolerance).length();

    double acceleration_time = robot_constants.robot_max_speed_m_per_s /
                               robot_constants.robot_max_acceleration_m_per_s_2;
    // x = v*t + 1/2*a*t^2, v = initial velocity = 0
    double acceleration_distance = 0.5 *
                                   robot_constants.robot_max_acceleration_m_per_s_2 *
                                   std::pow(acceleration_time, 2);
    double time_at_max_vel = (distance_to_dest - 2 * acceleration_distance) /
                             robot_constants.robot_max_speed_m_per_s;

    double travel_time = 2 * acceleration_time + time_at_max_vel;

    EXPECT_EQ(
        Duration::fromSeconds(travel_time),
        getTimeToPositionForRobot(robot_location, target_location,
                                  robot_constants.robot_max_speed_m_per_s,
                                  robot_constants.robot_max_acceleration_m_per_s_2, 0.5));
}
