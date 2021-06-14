#include "software/geom/angle_map.h"

#include <gtest/gtest.h>

TEST(AngleMapTest, add_obstacle_angle_segment_takes_entire_angle_map)
{
    Angle top_angle    = Angle::half();
    Angle bottom_angle = Angle::zero();
    AngleMap angle_map = AngleMap(top_angle, bottom_angle, 1);

    AngleSegment obstacle_angle_seg = AngleSegment(top_angle, bottom_angle);
    angle_map.addNonViableAngleSegment(obstacle_angle_seg);

    EXPECT_EQ(0, angle_map.getBiggestViableAngleSegment().getDeltaInDegrees());
}

TEST(AngleMapTest, add_obstacle_angle_segment_takes_lower_half_angle_map)
{
    Angle top_angle    = Angle::half();
    Angle bottom_angle = Angle::zero();
    AngleMap angle_map = AngleMap(top_angle, bottom_angle, 1);

    AngleSegment obstacle_angle_seg = AngleSegment(Angle::quarter(), Angle::zero());
    angle_map.addNonViableAngleSegment(obstacle_angle_seg);

    EXPECT_EQ(90, angle_map.getBiggestViableAngleSegment().getDeltaInDegrees());
}

TEST(AngleMapTest, add_obstacle_angle_segment_takes_upper_half_angle_map)
{
    Angle top_angle    = Angle::half();
    Angle bottom_angle = Angle::zero();
    AngleMap angle_map = AngleMap(top_angle, bottom_angle, 1);

    AngleSegment obstacle_angle_seg = AngleSegment(Angle::half(), Angle::quarter());
    angle_map.addNonViableAngleSegment(obstacle_angle_seg);

    EXPECT_EQ(90, angle_map.getBiggestViableAngleSegment().getDeltaInDegrees());
}

TEST(AngleMapTest, add_obstacle_angle_segment_contained_within_another)
{
    Angle top_angle    = Angle::half();
    Angle bottom_angle = Angle::zero();
    AngleMap angle_map = AngleMap(top_angle, bottom_angle, 1);

    AngleSegment obstacle_angle_seg = AngleSegment(Angle::quarter(), Angle::zero());
    angle_map.addNonViableAngleSegment(obstacle_angle_seg);

    AngleSegment overlapping_angle_seg =
        AngleSegment(Angle::fromDegrees(45), Angle::zero());
    angle_map.addNonViableAngleSegment(overlapping_angle_seg);

    EXPECT_EQ(90, angle_map.getBiggestViableAngleSegment().getDeltaInDegrees());
}

TEST(AngleMapTest, add_obstacle_angle_segment_overlaps_with_another)
{
    Angle top_angle    = Angle::half();
    Angle bottom_angle = Angle::zero();
    AngleMap angle_map = AngleMap(top_angle, bottom_angle, 1);

    AngleSegment obstacle_angle_seg = AngleSegment(Angle::quarter(), Angle::zero());
    angle_map.addNonViableAngleSegment(obstacle_angle_seg);

    AngleSegment overlapping_angle_seg =
        AngleSegment(Angle::fromDegrees(135), Angle::fromDegrees(45));
    angle_map.addNonViableAngleSegment(overlapping_angle_seg);

    EXPECT_EQ(45, angle_map.getBiggestViableAngleSegment().getDeltaInDegrees());
}
