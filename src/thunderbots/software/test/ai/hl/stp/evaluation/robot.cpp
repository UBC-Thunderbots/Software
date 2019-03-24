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
