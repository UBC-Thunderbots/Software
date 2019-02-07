/**
 * This file contains the unit tests for evaluation function playerOrientationWithinAngleThresholdOfTarget in player.cpp
 */

#include "ai/hl/stp/evaluation/player.h"

#include <gtest/gtest.h>

using namespace AI::HL::STP;

TEST(PlayerOrientationWithinAngleThresholdOfTargetTest, orientation_in_threshold_orientation_45_threshold_60)
{
    Point position = Point(0,0);
    Angle orientation = Angle::ofDegrees(0);
    Point target = Point(1,1);
    Angle threshold = Angle::ofDegrees(60);

    EXPECT_TRUE(Evaluation::playerOrientationWithinAngleThresholdOfTarget(position,orientation,target,threshold));
}


TEST(PlayerOrientationWithinAngleThresholdOfTargetTest, orientation_not_in_threshold_orientation_45_threshold_30)
{
    Point position = Point(0,0);
    Angle orientation = Angle::ofDegrees(0);
    Point target = Point(1,1);
    Angle threshold = Angle::ofDegrees(30);

    EXPECT_FALSE(Evaluation::playerOrientationWithinAngleThresholdOfTarget(position,orientation,target,threshold));

}

TEST(PlayerOrientationWithinAngleThresholdOfTargetTest, orientation_not_in_threshold_orientation_45_threshold_45)
{
    Point position = Point(0,0);
    Angle orientation = Angle::ofDegrees(0);
    Point target = Point(1,1);
    Angle threshold = Angle::ofDegrees(45);

    EXPECT_FALSE(Evaluation::playerOrientationWithinAngleThresholdOfTarget(position,orientation,target,threshold));

}

TEST(PlayerOrientationWithinAngleThresholdOfTargetTest, orientation_in_threshold_orientation_135_threshold_150)
{
    Point position = Point(0,0);
    Angle orientation = Angle::ofDegrees(0);
    Point target = Point(-1,1);
    Angle threshold = Angle::ofDegrees(150);

    EXPECT_TRUE(Evaluation::playerOrientationWithinAngleThresholdOfTarget(position,orientation,target,threshold));

}

TEST(PlayerOrientationWithinAngleThresholdOfTargetTest, orientation_in_threshold_orientation_135_threshold_90)
{
    Point position = Point(0,0);
    Angle orientation = Angle::ofDegrees(0);
    Point target = Point(-1,1);
    Angle threshold = Angle::ofDegrees(90);

    EXPECT_FALSE(Evaluation::playerOrientationWithinAngleThresholdOfTarget(position,orientation,target,threshold));

}



int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

