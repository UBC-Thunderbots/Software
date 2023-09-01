#include "software/geom/pose.h"

#include <gtest/gtest.h>

TEST(CreatePoseTests, pose_default_constructor_test)
{
    Pose p = Pose();
    EXPECT_EQ(Point(0, 0), p.position());
    EXPECT_EQ(Angle::zero(), p.orientation());
}

TEST(CreatePoseTests, pose_specific_constructor_test)
{
    Pose p = Pose(Point(1, 2), Angle::quarter());
    EXPECT_EQ(Point(1, 2), p.position());
    EXPECT_EQ(Angle::quarter(), p.orientation());
}

TEST(PoseOperatorTests, pose_equality_test)
{
    Pose p = Pose(Point(1, 2), Angle::quarter());
    Pose q = Pose(Point(1, 2), Angle::quarter());

    EXPECT_TRUE(p == q);
    EXPECT_FALSE(p != q);
}

TEST(PoseOperatorTests, pose_inequality_test)
{
    Pose p = Pose(Point(1, 2), Angle::quarter());
    Pose q = Pose(Point(3, 2), Angle::quarter());

    EXPECT_FALSE(p == q);
    EXPECT_TRUE(p != q);
}
