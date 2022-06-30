#include "software/ai/evaluation/time_to_travel.h"

#include <gtest/gtest.h>

#include "software/test_util/equal_within_tolerance.h"

class TimeToTravel : public ::testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
};

TEST_F(TimeToTravel, getTimeToTravelDistance_already_at_dest)
{
    EXPECT_EQ(Duration::fromSeconds(0), getTimeToTravelDistance(0, 2.0, 3.0));
}

TEST_F(TimeToTravel, getTimeToTravelDistance_reaches_max_velocity)
{
    // Check that the robot reaches the dest at the expected time when
    // it has enough time that it accelerates up to it's maximum velocity
    // We set the distance here such that it the robot *should* always have time to
    // get to the max velocity, unless our robots suddenly get *much* faster

    double distance_to_dest = 50;

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
        getTimeToTravelDistance(distance_to_dest, robot_constants.robot_max_speed_m_per_s,
                                robot_constants.robot_max_acceleration_m_per_s_2)));
}

TEST_F(TimeToTravel,
       getTimeToPositionForRobot_with_equal_positive_initial_and_final_velocity)
{
    // Check that the robot reaches the dest in the expected time when
    // it has enough time that it accelerates up to it's maximum velocity and
    // decelerates to final velocity.

    // Equal initial and final velocities
    double initial_velocity = 2.0;
    double max_accel        = 3.0;
    double max_vel          = 4.0;
    double distance_to_dest = 5;

    double acceleration_time = (max_vel - initial_velocity) / max_accel;

    // x = v*t + 1/2*a*t^2, v = initial velocity
    double acceleration_distance = initial_velocity * acceleration_time +
                                   0.5 * max_accel * std::pow(acceleration_time, 2);
    double time_at_max_vel = (distance_to_dest - 2 * acceleration_distance) / max_vel;

    // Acceleration and deceleration time are equal since initial velocity is equal
    // to final velocity
    double travel_time_expected = 2 * acceleration_time + time_at_max_vel;

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Duration::fromSeconds(travel_time_expected),
        getTimeToTravelDistance(distance_to_dest, max_vel, max_accel, initial_velocity,
                                initial_velocity)));
}

TEST_F(TimeToTravel, getTimeToPositionForRobot_with_robot_not_reaching_max_velocity)
{
    // Check that the robot reaches the dest in the expected time when
    // it does not have enough time to accelerate up to it's maximum velocity
    double initial_velocity = 1.0;
    double final_velocity   = 3.0;
    // Relatively low acceleration so max velocity is not reachable
    double max_accel = 1.0;
    double max_vel   = 4.0;
    double distance  = 10.0;

    // Calculated graphically through Desmos, work can be found here:
    // https://www.desmos.com/calculator/4glnzfc45x
    double travel_time_expected = 3.74596669241;
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Duration::fromSeconds(travel_time_expected),
        getTimeToTravelDistance(distance, max_vel, max_accel, initial_velocity,
                                final_velocity)));
}

TEST_F(TimeToTravel, getTimeToPositionForRobot_with_initial_velocity_away_from_dest)
{
    // Check that the robot reaches the dest in the at the expected time when
    // it has plenty of space to accelerate to max velocity and begins with its
    // initial velocity away from the destination

    // Initial velocity away from destination
    double initial_vel = -1.0;
    double final_vel   = 3.0;
    double max_accel   = 2.0;
    double max_vel     = 4.0;
    double distance    = 30.0;

    double acceleration_time = (max_vel - initial_vel) / max_accel;
    // d = (Vi + Vf) / 2 * t
    double acceleration_distance = (initial_vel + max_vel) / 2 * acceleration_time;

    double deceleration_time = (final_vel - max_vel) / -max_accel;
    // d = (Vi + Vf) / 2 * t
    double deceleration_distance = (max_vel + final_vel) / 2 * deceleration_time;

    double time_at_max_vel =
        (distance - acceleration_distance - deceleration_distance) / max_vel;

    double travel_time_expected = acceleration_time + time_at_max_vel + deceleration_time;

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Duration::fromSeconds(travel_time_expected),
        getTimeToTravelDistance(distance, max_vel, max_accel, initial_vel, final_vel)));
}

TEST_F(
    TimeToTravel,
    getTimeToTravelDistance_with_final_velocity_not_achievable_in_distance_and_greater_than_initial_velocity)
{
    // Check that the robot reaches the dest with in an expected time frame when
    // it does not have enough space to accelerate to the final velocity

    // Equal initial and final velocities, both in the direction of the destination
    double initial_vel = 0.0;
    double final_vel   = 4.0;
    double max_accel   = 1.0;
    double max_vel     = 4.0;
    double distance    = 1.0;

    // The robot is accelerating throughout the path (since the final velocity is too
    // great for it to reach within the distance) t = (-Vi + sqrt(Vi^2 + 2 * a * d)) / a
    double acceleration_time =
        (-initial_vel + std::sqrt(std::pow(initial_vel, 2) + 2 * max_accel * distance)) /
        max_accel;

    // For the upper bound, just choose a time that's much greater than we would expect
    double max_time_to_dest = 5.0;

    Duration t =
        getTimeToTravelDistance(distance, max_vel, max_accel, initial_vel, final_vel);
    EXPECT_LE(Duration::fromSeconds(acceleration_time), t);
    EXPECT_GE(Duration::fromSeconds(max_time_to_dest), t);
}

TEST_F(
    TimeToTravel,
    getTimeToPositionForRobot_with_final_velocity_not_achievable_in_distance_and_smaller_than_initial_velocity)
{
    // Check that the robot reaches the dest with in an expected time frame when
    // it does not have enough space to accelerate to the final velocity

    // Equal initial and final velocities, both in the direction of the destination
    double initial_vel = 4.0;
    double final_vel   = 0.0;
    double max_accel   = 1.0;
    double max_vel     = 4.0;
    double dist_required_to_reach_v_f =
        std::abs(std::pow(final_vel, 2) - std::pow(initial_vel, 2)) / (2 * max_accel);
    double distance = dist_required_to_reach_v_f;

    // The robot is decelerating throughout the path (since the final velocity is too
    // small for it to reach within the distance) t = (-Vi + sqrt(Vi^2 + 2 * a * d)) / a,
    // with a being negative due to deceleration

    // For the upper bound, just choose a time that's much greater than we would expect
    double max_time_to_dest = 5.0;

    Duration t =
        getTimeToTravelDistance(distance, max_vel, max_accel, initial_vel, final_vel);
    EXPECT_LE(Duration::fromSeconds(0), t);
    EXPECT_GE(Duration::fromSeconds(max_time_to_dest), t);
}
