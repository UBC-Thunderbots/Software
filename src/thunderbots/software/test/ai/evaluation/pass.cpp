/**
 * Tests for the pass evaluation functions
 */

#include <gtest/gtest.h>

#include "ai/evaluation/pass.h"
#include "../shared/constants.h"

using namespace AI::Evaluation;

TEST(PassingEvaluationTest, getTimeToOrientationForRobot_robot_at_desired_angle)
{
Angle target_angle = Angle::half();
Robot robot(0, {1, 1}, Vector(0, 0), Angle::half(), AngularVelocity::ofDegrees(0),
            Timestamp::fromSeconds(0));

EXPECT_EQ(Duration::fromSeconds(0),
        getTimeToOrientationForRobot(robot, target_angle, 4 * M_PI, 10.0));
}

TEST(PassingEvaluationTest, getTimeToOrientationForRobot_robot_opposite_to_desired_angle)
{
// Because we can't guarantee that the robot angular acceleration isn't going to
// increase to the point where we can't reach the the max angular speed within
// a half rotation, this is just a more loose test that checks if we got there
// within roughly the expected time. Any more strict test would have to
// basically re-write the function, which would be a bit pointless

Angle target_angle = Angle::zero();
Robot robot(0, {1, 1}, Vector(0, 0), Angle::half(), AngularVelocity::ofDegrees(0),
            Timestamp::fromSeconds(0));

// Figure out a lower bound on the time required, based on us being able to constantly
// accelerate at the max acceleration
// s = ut + 1/2 * at^2, u = 0, s = pi/2
// t = sqrt(2*s/a)
double min_time_to_rotate = 0.56;

// For the upper bound, just choose a time that's much greater then we would expect
double max_time_to_rotate = 4.0;

Duration t = getTimeToOrientationForRobot(robot, target_angle, 4 * M_PI, 10);
EXPECT_LE(Duration::fromSeconds(min_time_to_rotate),
        getTimeToOrientationForRobot(robot, target_angle, 4 * M_PI, 10));
EXPECT_GE(Duration::fromSeconds(max_time_to_rotate),
        getTimeToOrientationForRobot(robot, target_angle, 4 * M_PI, 10));
}

TEST(PassingEvaluationTest, getTimeToPositionForRobot_already_at_dest)
{
Point dest(1, 1);
Robot robot(0, dest, Vector(0, 0), Angle::ofDegrees(0), AngularVelocity::ofDegrees(0),
            Timestamp::fromSeconds(0));

EXPECT_EQ(Duration::fromSeconds(0), getTimeToPositionForRobot(robot, dest, 2.0, 3.0));
}

TEST(PassingEvaluationTest, getTimeToPositionForRobot_reaches_max_velocity)
{
// Check that the robot reaches the dest in the at the expected time when
// it has enough time that it accelerates up to it's maximum velocity
// We set the distance here such that it the robot *should* always have time to
// get to the max velocity, unless our robots suddenly get *much* faster

Point dest(1, 1);
Point robot_location(40, 40);
Robot robot(0, robot_location, Vector(0, 0), Angle::ofDegrees(0),
            AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0));

double distance_to_dest = (robot_location - dest).len();

double acceleration_time = ROBOT_MAX_SPEED_METERS_PER_SECOND /
                           ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
// x = v*t + 1/2*a*t^2, v = initial velocity = 0
double acceleration_distance = 0.5 *
                               ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED *
                               std::pow(acceleration_time, 2);
double time_at_max_vel = (distance_to_dest - 2 * acceleration_distance) /
                         ROBOT_MAX_SPEED_METERS_PER_SECOND;

double travel_time = 2 * acceleration_time + time_at_max_vel;

EXPECT_EQ(Duration::fromSeconds(travel_time),
        getTimeToPositionForRobot(robot, dest, 2.0, 3.0));
}
