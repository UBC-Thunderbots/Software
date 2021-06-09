#include "software/geom/angle_segment.h"

#include <gtest/gtest.h>

TEST(AngleSegmentTest, get_delta_zero_angle)
{
    AngleSegment angle_seg = AngleSegment(Angle::half(), Angle::zero());

    EXPECT_EQ(180, angle_seg.getDeltaInDegrees());
}

TEST(AngleSegmentTest, get_delta_both_positive)
{
    AngleSegment angle_seg = AngleSegment(Angle::half(), Angle::quarter());

    EXPECT_EQ(90, angle_seg.getDeltaInDegrees());
}

TEST(AngleSegmentTest, get_delta_one_positive_one_negative)
{
    AngleSegment angle_seg = AngleSegment(Angle::quarter(), Angle::fromDegrees(-90));

    EXPECT_EQ(180, angle_seg.getDeltaInDegrees());
}

TEST(AngleSegmentTest, get_delta_both_negative)
{
    AngleSegment angle_seg =
        AngleSegment(Angle::fromDegrees(-90), Angle::fromDegrees(-45));

    EXPECT_EQ(45, angle_seg.getDeltaInDegrees());
}

TEST(AngleSegmentTest, equality_top_equals_bottom_doesnt)
{
    AngleSegment angle_seg       = AngleSegment(Angle::quarter(), Angle::threeQuarter());
    AngleSegment other_angle_seg = AngleSegment(Angle::quarter(), Angle::zero());

    EXPECT_EQ(angle_seg, other_angle_seg);
}

TEST(AngleSegmentTest, equality_bottom_equals_top_doesnt)
{
    AngleSegment angle_seg       = AngleSegment(Angle::quarter(), Angle::threeQuarter());
    AngleSegment other_angle_seg = AngleSegment(Angle::half(), Angle::zero());

    EXPECT_NE(angle_seg, other_angle_seg);
}

TEST(AngleSegmentTest, equality_bottom_and_top_equals)
{
    AngleSegment angle_seg       = AngleSegment(Angle::quarter(), Angle::threeQuarter());
    AngleSegment other_angle_seg = AngleSegment(Angle::quarter(), Angle::threeQuarter());

    EXPECT_EQ(angle_seg, other_angle_seg);
}

TEST(AngleSegmentTest, top_less_than)
{
    AngleSegment angle_seg       = AngleSegment(Angle::zero(), Angle::threeQuarter());
    AngleSegment other_angle_seg = AngleSegment(Angle::quarter(), Angle::threeQuarter());

    EXPECT_LT(angle_seg, other_angle_seg);
}

TEST(AngleSegmentTest, top_greater_than)
{
    AngleSegment angle_seg       = AngleSegment(Angle::quarter(), Angle::threeQuarter());
    AngleSegment other_angle_seg = AngleSegment(Angle::zero(), Angle::threeQuarter());

    EXPECT_GT(angle_seg, other_angle_seg);
}
