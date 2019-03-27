/**
 * This file contains the unit tests for evaluation functions
 * in shot_score.cpp
 */

#include <gtest/gtest.h>

#include "ai/hl/stp/evaluation/shot_score.h"

// TEST(RobotEvaluationTest,
//        orientation_not_in_threshold_facing_150_target_minus_150_threshold_30)
//{
// Point position    = Point(0, 0);
// Angle orientation = Angle::ofDegrees(150);
// Point target      = Point(-2, -1);
// Angle threshold   = Angle::ofDegrees(30);
//
// EXPECT_FALSE(Evaluation::robotOrientationWithinAngleThresholdOfTarget(
//        position, orientation, target, threshold));
//}