#include "software/ai/evaluation/pass.h"

#include <gtest/gtest.h>

#include "software/test_util/equal_within_tolerance.h"

class PassingEvaluationTest : public ::testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2015RobotConstants();
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
    EXPECT_EQ(Duration::fromSeconds(0), getTimeToPositionForRobot(dest, dest, 2.0, 3.0));
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

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Duration::fromSeconds(travel_time),
        getTimeToPositionForRobot(robot_location, dest, 2.0, 3.0)));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_reaches_max_velocity_with_tolerance)
{
    // Check that the robot reaches the dest in the expected time when
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

    double travel_time_expected = 2 * acceleration_time + time_at_max_vel;

    EXPECT_TRUE(TestUtil::equalWithinTolerance(Duration::fromSeconds(travel_time_expected),
                                               getTimeToPositionForRobot(robot_location, target_location, 2.0, 3.0, 0.5)));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_with_initial_velocity_and_not_final_velocity)
{
    // Check that the robot reaches the dest in the expected time when
    // it has enough time that it accelerates up to it's maximum velocity and
    // decelerates to final velocity.

    Point robot_location(0, 0);
    Point target_location(3, 4);

    // Equal initial and final velocities, both in the direction of the destination
    double initial_velocity = 2.0;
    double max_accel = 3.0;
    double max_vel = 4.0;

    Vector distance_to_dest_vector = target_location - robot_location;
    double distance_to_dest = distance_to_dest_vector.length();

    double acceleration_time = (max_vel - initial_velocity) / max_accel;

    // x = v*t + 1/2*a*t^2, v = initial velocity
    double acceleration_distance = initial_velocity * acceleration_time + 0.5 *
                                   max_accel * std::pow(acceleration_time, 2);
    double time_at_max_vel = (distance_to_dest - 2 * acceleration_distance) /
                             max_vel;

    // Acceleration and deceleration time are equal since initial velocity is equal
    // to final velocity
    double travel_time_expected = 2 * acceleration_time + time_at_max_vel;

    EXPECT_TRUE(TestUtil::equalWithinTolerance(Duration::fromSeconds(travel_time_expected),
                                               getTimeToPositionForRobot(robot_location, target_location, max_vel, max_accel, 0,
                                                                         distance_to_dest_vector.normalize(initial_velocity),
                                                                         distance_to_dest_vector.normalize(initial_velocity))));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_with_robot_not_reaching_max_velocity)
{
    // Check that the robot reaches the dest in the expected time when
    // it does not have enough time to accelerate up to it's maximum velocity

    Point robot_location(0, 0);
    // Target location 10 meters away
    Point target_location(6, 8);
    Vector distance_to_dest_vector = target_location - robot_location;

    // Equal initial and final velocities, both in the direction of the destination
    double initial_velocity = 1.0;
    double final_velocity = 3.0;
    double max_accel = 1.0;
    double max_vel = 4.0;

    // Calculated graphically through Desmos, work can be found here:
    // https://www.desmos.com/calculator/4glnzfc45x
    double travel_time_expected = 3.74596669241;
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Duration::fromSeconds(travel_time_expected),
                                               getTimeToPositionForRobot(robot_location, target_location, max_vel, max_accel, 0,
                                                                         distance_to_dest_vector.normalize(initial_velocity),
                                                                         distance_to_dest_vector.normalize(final_velocity))));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_with_initial_velocity_away_from_dest)
{
    // Check that the robot reaches the dest in the at the expected time when
    // it has plenty of space to accelerate to max velocity and begins with its
    // initial velocity away from the destination

    Point robot_location(0, 0);
    Point target_location(0, 30);

    // Equal initial and final velocities, both in the direction of the destination
    double initial_vel = -1.0;
    double final_vel = 3.0;
    double max_accel = 2.0;
    double max_vel = 4.0;

    Vector distance_to_dest_vector = target_location - robot_location;
    double distance_to_dest = distance_to_dest_vector.length();

    double acceleration_time = (max_vel - initial_vel) / max_accel;
    // d = (Vi + Vf) / 2 * t
    double acceleration_distance = (initial_vel + max_vel) / 2 * acceleration_time;

    double deceleration_time = (final_vel - max_vel) / -max_accel;
    // d = (Vi + Vf) / 2 * t
    double deceleration_distance = (max_vel + final_vel) / 2 * deceleration_time;

    double time_at_max_vel = (distance_to_dest - acceleration_distance - deceleration_distance) /
                             max_vel;

    double travel_time_expected = acceleration_time + time_at_max_vel + deceleration_time;

    EXPECT_TRUE(TestUtil::equalWithinTolerance(Duration::fromSeconds(travel_time_expected),
                                               getTimeToPositionForRobot(robot_location, target_location, max_vel, max_accel, 0,
                                                                         distance_to_dest_vector.normalize(initial_vel),
                                                                         distance_to_dest_vector.normalize(final_vel))));
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_with_final_velocity_not_achievable_in_distance_and_greater_than_initial_velocity)
{
    // Check that the robot reaches the dest with in an expected time frame when
    // it does not have enough space to accelerate to the final velocity

    Point robot_location(0, 0);
    Point target_location(0, 1);

    // Equal initial and final velocities, both in the direction of the destination
    double initial_vel = 0.0;
    double final_vel = 4.0;
    double max_accel = 1.0;
    double max_vel = 4.0;

    Vector distance_to_dest_vector = target_location - robot_location;
    double distance_to_dest = distance_to_dest_vector.length();

    // The robot is accelerating throughout the path (since the final velocity is too great for it
    // to reach within the distance)
    // t = (-Vi + sqrt(Vi^2 + 2 * a * d)) / a
    double acceleration_time = (-initial_vel + std::sqrt(std::pow(initial_vel, 2) + 2 * max_accel * distance_to_dest)) / max_accel;

    // For the upper bound, just choose a time that's much greater then we would expect
    double max_time_to_dest = 5.0;

    Duration t =
            getTimeToPositionForRobot(robot_location, target_location, max_vel, max_accel, 0,
                                      distance_to_dest_vector.normalize(initial_vel),
                                      distance_to_dest_vector.normalize(final_vel));
    EXPECT_LE(
            Duration::fromSeconds(acceleration_time),
            t);
    EXPECT_GE(
            Duration::fromSeconds(max_time_to_dest),
            t);
}

TEST_F(PassingEvaluationTest,
       getTimeToPositionForRobot_with_final_velocity_not_achievable_in_distance_and_smaller_than_initial_velocity)
{
    // Check that the robot reaches the dest with in an expected time frame when
    // it does not have enough space to accelerate to the final velocity

    Point robot_location(0, 0);
    Point target_location(0, 1);

    // Equal initial and final velocities, both in the direction of the destination
    double initial_vel = 4.0;
    double final_vel = 0.0;
    double max_accel = 1.0;
    double max_vel = 4.0;

    Vector distance_to_dest_vector = target_location - robot_location;
    double distance_to_dest = distance_to_dest_vector.length();

    // The robot is decelerating throughout the path (since the final velocity is too small for it
    // to reach within the distance)
    // t = (-Vi + sqrt(Vi^2 + 2 * a * d)) / a, with a being negative due to deceleration
    double acceleration_time = (-initial_vel + std::sqrt(std::pow(initial_vel, 2) - 2 * max_accel * distance_to_dest)) / -max_accel;

    // For the upper bound, just choose a time that's much greater then we would expect
    double max_time_to_dest = 5.0;

    Duration t =
            getTimeToPositionForRobot(robot_location, target_location, max_vel, max_accel, 0,
                                      distance_to_dest_vector.normalize(initial_vel),
                                      distance_to_dest_vector.normalize(final_vel));
    EXPECT_LE(
            Duration::fromSeconds(acceleration_time),
            t);
    EXPECT_GE(
            Duration::fromSeconds(max_time_to_dest),
            t);
}
