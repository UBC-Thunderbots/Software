/**
 * This file contains the unit tests for evaluation functions
 * in robot.cpp
 */

#include "ai/hl/stp/evaluation/robot.h"

#include <gtest/gtest.h>


TEST(RobotEvaluationTest, orientation_in_threshold_facing_0_target_45_threshold_60)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::ofDegrees(60);

    EXPECT_TRUE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}


TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_45_threshold_30)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::ofDegrees(30);

    EXPECT_FALSE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_45_threshold_45)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(0);
    Point target      = Point(1, 1);
    Angle threshold   = Angle::ofDegrees(45);

    EXPECT_FALSE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest, orientation_in_threshold_facing_0_target_135_threshold_150)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(0);
    Point target      = Point(-1, 1);
    Angle threshold   = Angle::ofDegrees(150);

    EXPECT_TRUE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest, orientation_not_in_threshold_facing_0_target_135_threshold_90)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(0);
    Point target      = Point(-1, 1);
    Angle threshold   = Angle::ofDegrees(90);

    EXPECT_FALSE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest,
     orientation_in_threshold_facing_150_target_minus_150_threshold_90)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(150);
    Point target      = Point(-2, -1);
    Angle threshold   = Angle::ofDegrees(90);

    EXPECT_TRUE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest,
     orientation_not_in_threshold_facing_150_target_minus_150_threshold_30)
{
    Point position    = Point(0, 0);
    Angle orientation = Angle::ofDegrees(150);
    Point target      = Point(-2, -1);
    Angle threshold   = Angle::ofDegrees(30);

    EXPECT_FALSE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
        position, orientation, target, threshold));
}

TEST(RobotEvaluationTest, has_possession_directly_in_front_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_TRUE(Evaluation::robotHasPossession(ball, robot));
}

TEST(RobotEvaluationTest, has_possession_ball_to_side_of_robot)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::half(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_FALSE(Evaluation::robotHasPossession(ball, robot));
}

TEST(RobotEvaluationTest, has_possession_robot_moving_ball_in_dribbler)
{
    Point ball_position  = Point(0.07, 0);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(1, 1), Angle::zero(),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(Evaluation::robotHasPossession(ball, robot));
}

TEST(RobotEvaluationTest, has_possession_ball_far_away_from_robot)
{
    Point ball_position  = Point(-1, -2);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_FALSE(Evaluation::robotHasPossession(ball, robot));
}

TEST(RobotEvaluationTest, has_possession_ball_slightly_off_center_but_still_on_dribbler)
{
    Point ball_position  = Point(0.07, 0.005);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        timestamp);

    EXPECT_TRUE(Evaluation::robotHasPossession(ball, robot));
}



TEST(RobotEvaluationTest, has_possession_robot_on_angle_with_ball_in_dribbler)
{
    Point ball_position  = Point(0.035, 0.06);
    Vector ball_velocity = Vector(0, 0);
    Timestamp timestamp  = Timestamp::fromSeconds(0);
    Ball ball            = Ball(ball_position, ball_velocity, timestamp);

    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::ofDegrees(59.74356),
                        AngularVelocity::zero(), timestamp);

    EXPECT_TRUE(Evaluation::robotHasPossession(ball, robot));
}
