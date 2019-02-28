/**
 * This file contains unit tests passing evaluation functions
 */

#include "ai/passing/evaluation.h"
#include "../shared/constants.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

using namespace AI::Passing;

TEST(PassingEvaluationTest, getFriendlyCapability_no_robots_on_team){
    Team team(Duration::fromSeconds(10));
    team.updateRobots({});
    Pass pass({0,0}, {1,1}, 10, Timestamp::fromSeconds(10));

    // If there are no robots on the team, then there is no way we can receive a pass
    EXPECT_EQ(0, getFriendlyCapability(team, pass));
}

TEST(PassingEvaluationTest, getFriendlyCapability_one_robot_near_pass_one_far_away){
    // Test getting friendly capability for a team with two robots, one near the pass
    // reception point and the other far away
    Team team(Duration::fromSeconds(10));
    team.updateRobots({
        Robot(0, {15.5, -10}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0)),
        Robot(1, {100, -100}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))
    });
    Pass pass({0,0}, {15, -10.1}, 10, Timestamp::fromSeconds(10));

    // There should be a very high probability that we can receive this pass
    EXPECT_LE(0.9, getFriendlyCapability(team, pass));
    EXPECT_GE(1, getFriendlyCapability(team, pass));
}

TEST(PassingEvaluationTest, getFriendlyCapability_multiple_robots_all_too_far_from_reception_point){
    // Test case where there are lots of robots far away from the reception point and
    // there *is not* enough time for them to get to the reception point
    Team team(Duration::fromSeconds(10));
    team.updateRobots({
                              Robot(0, {15.5, -10}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0)),
                              Robot(1, {100, -100}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))
                      });
    Pass pass({0,0}, {1, 1}, 10, Timestamp::fromSeconds(1));

    EXPECT_GE(0.1, getFriendlyCapability(team, pass));
    EXPECT_LE(0, getFriendlyCapability(team, pass));
}

TEST(PassingEvaluationTest, getFriendlyCapability_multiple_robots_close_enough_to_reception_point){
    // Test case where there are lots of robots far away from the reception point, but
    // when *there is* enough time for them to reach the receive point
    Team team(Duration::fromSeconds(10));
    team.updateRobots({
                              Robot(0, {110, 110}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(5)),
                              Robot(1, {100, -100}, {0,0}, Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(5))
                      });
    Pass pass({100,100}, {120, 105}, 1, Timestamp::fromSeconds(10));

    EXPECT_LE(0.9, getFriendlyCapability(team, pass));
    EXPECT_GE(1, getFriendlyCapability(team, pass));
}

TEST(PassingEvaluationTest, getFriendlyCapability_single_robot_cant_turn_in_time){
    // Test case where this is one robot, but it is turned in the wrong direction and
    // will not be able to turn in time to receive the pass
    Team team(Duration::fromSeconds(10));
    team.updateRobots({
                              Robot(0, {1, 0}, {0,0}, Angle::quarter(), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0))
                      });
    Pass pass({0,0}, {-1, -1}, 3, Timestamp::fromSeconds(0.1));

    EXPECT_GE(0.1, getFriendlyCapability(team, pass));
    EXPECT_LE(0, getFriendlyCapability(team, pass));
}

// TODO: Test case where a robot is facing away from a pass and there is no way it can turn in time

TEST(PassingEvaluationTest, getTimeToOrientationForRobot_robot_at_desired_angle){
    Angle target_angle = Angle::half();
    Robot robot(0, {1,1}, Vector(0,0), Angle::half(), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0));

    EXPECT_EQ(Timestamp::fromSeconds(0), getTimeToOrientationForRobot(robot, target_angle));
}

TEST(PassingEvaluationTest, getTimeToOrientationForRobot_robot_opposite_to_desired_angle){
    // Because we can't guarantee that the robot angular acceleration isn't going to
    // increase to the point where we can't reach the the max angular speed within
    // a half rotation, this is just a more loose test that checks if we got there
    // within roughly the expected time. Any more strict test would have to
    // basically re-write the function, which would be a bit pointless

    Angle target_angle = Angle::zero();
    Robot robot(0, {1,1}, Vector(0,0), Angle::half(), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0));

    // Figure out a lower bound on the time required, based on us being able to constantly
    // accelerate at the max acceleration
    double min_time_to_rotate = ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND / ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED;

    // For the upper bound, just choose a time that's much greater then we would expect
    double max_time_to_rotate = 4.0;

    EXPECT_LE(Timestamp::fromSeconds(min_time_to_rotate), getTimeToOrientationForRobot(robot, target_angle));
    EXPECT_GE(Timestamp::fromSeconds(max_time_to_rotate), getTimeToOrientationForRobot(robot, target_angle));
}

TEST(PassingEvaluationTest, getTimeToPositionForRobot_already_at_dest){
    Point dest(1,1);
    Robot robot(0, dest, Vector(0,0), Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0));

    EXPECT_EQ(Timestamp::fromSeconds(0), getTimeToPositionForRobot(robot, dest));
}

TEST(PassingEvaluationTest, getTimeToPositionForRobot_reaches_max_velocity){
    // Check that the robot reaches the dest in the at the expected time when
    // it has enough time that it accelerates up to it's maximum velocity
    // We set the distance here such that it the robot *should* always have time to
    // get to the max velocity, unless our robots suddenly get *much* faster

    Point dest(1,1);
    Point robot_location(40, 40);
    Robot robot(0, robot_location, Vector(0,0), Angle::ofDegrees(0), AngularVelocity::ofDegrees(0), Timestamp::fromSeconds(0));

    double distance_to_dest = (robot_location - dest).len();

    double acceleration_time = ROBOT_MAX_SPEED_METERS_PER_SECOND / ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    // x = v*t + 1/2*a*t^2, v = initial velocity = 0
    double acceleration_distance = 0.5 * ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*std::pow(acceleration_time, 2);
    double time_at_max_vel = (distance_to_dest - 2*acceleration_distance)/ROBOT_MAX_SPEED_METERS_PER_SECOND;

    double travel_time = 2*acceleration_time + time_at_max_vel;

    EXPECT_EQ(Timestamp::fromSeconds(travel_time), getTimeToPositionForRobot(robot, dest));
}

TEST(PassingEvaluationTest, getStaticPositionQuality_on_field_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that the static quality is basically 0 at the edge of the field
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, -3.0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, 3.0)), 0.13);
}

TEST(PassingEvaluationTest, getStaticPositionQuality_near_own_goal_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that we have a static quality of almost 0 near our goal
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.0, 0)), 0.14);
}

TEST(PassingEvaluationTest, getStaticPositionQuality_near_enemy_goal_quality)
{
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that we have a large static quality near the enemy goal
    EXPECT_GE(getStaticPositionQuality(f, Point(3.0, 0)), 0.80);

    // But we should have basically 0 static quality too close to the enemy goal,
    // as there is a defense area around the net that we cannot pass to
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, 1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.3, -1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, 1.9)), 0.0, 0.1);
    EXPECT_NEAR(getStaticPositionQuality(f, Point(4.4, -1.9)), 0.0, 0.1);
}

TEST(PassingEvaluationTest, rectangleSigmoid_value_in_rectangle_centered_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {0, 0}, 0.1), 0.982);
}

TEST(PassingEvaluationTest, rectangleSigmoid_value_in_rectangle_offset_rectangle)
{
    Rectangle r1(Point(-2, -1), Point(0, 3));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {-1, 1}, 0.1), 0.9);
}

TEST(PassingEvaluationTest, rectangleSigmoid_values_outside_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that values off in x are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.2, 0}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.2, 0}, 0.1), 0.1);

    // Check that values off in y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {0, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {0, 2.1}, 0.1), 0.1);

    // Check that values off in x and y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, 2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, 2.1}, 0.1), 0.1);
}

TEST(PassingEvaluationTest, circleSigmoid_value_in_circle_centered_circle)
{
    Circle circle(Point(0, 0), 1);

    EXPECT_GE(circleSigmoid(circle, {0, 0}, 0.1), 0.982);
}

TEST(PassingEvaluationTest, circleSigmoid_value_in_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_GE(circleSigmoid(circle, {1, -1}, 0.1), 0.982);
}

TEST(PassingEvaluationTest, circleSigmoid_value_on_circle_edge)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_EQ(circleSigmoid(circle, {0, -1}, 0.1), 0.5);
    EXPECT_NEAR(circleSigmoid(circle, {1 + std::sqrt(2) / 2, -1 - std::sqrt(2) / 2}, 0.1),
                0.5, 0.01);
}

TEST(PassingEvaluationTest, circleSigmoid_value_outside_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    // Check that values off in x are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, 0}, 0.1), 0.018);

    // Check that values off in y are basically 0
    EXPECT_LE(circleSigmoid(circle, {0, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0, -2.2}, 0.1), 0.018);

    // Check that values off in x and y are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, -1.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0.2, -0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {-2.2, 1.2}, 0.1), 0.018);
}

TEST(PassingEvaluationTest, sigmoid_sig_width_is_respected)
{
    // Test the value at sig_width/2 is 0.982
    EXPECT_NEAR(sigmoid(5, 0, 10), 0.982, 0.0001);

    // Test the value at -sig_width/2 is 0.018
    EXPECT_NEAR(sigmoid(-5, 0, 10), 0.018, 0.001);
}

TEST(PassingEvaluationTest, sigmoid_offset)
{
    // Test that the value at 0 is 0.5 if no offset
    EXPECT_DOUBLE_EQ(0.5, sigmoid(0, 0, 1));

    // Test that the value at the offset is 0.5
    EXPECT_DOUBLE_EQ(0.5, sigmoid(2, 2, 1));
    EXPECT_DOUBLE_EQ(0.5, sigmoid(-2, -2, 1));
}

TEST(PassingEvaluationTest, sigmoid_negating_sig_width_flips_sigmoid)
{
    // Test that negating sig_width inverts the sigmoid
    EXPECT_NEAR(sigmoid(-5, 0, -10), 0.982, 0.0001);
    EXPECT_NEAR(sigmoid(5, 0, -10), 0.018, 0.0001);
}

