#include "software/new_geom/util/multiple_rays.h"

#include <gtest/gtest.h>

// Check the case where both rays intersect the segment only once (forming an intersection
// segment within the segment)
TEST(MultipleRaysTest, test_segment_intersect_with_existing_segment)
{
    Ray ray1 = Ray(Point(-1, 0), Vector(0, 1));
    Ray ray2 = Ray(Point(1, 0), Vector(0, 1));

    Segment segment = Segment(Point(-5, 4), Point(5, 4));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(Segment(Point(-1, 4), Point(1, 4)), intersecting_segment.value());
}

// Test that no segment is returned if both rays do not intersect the segment, and the
// area between the rays do not enclose the segment
TEST(MultipleRaysTest, test_segment_intersect_both_rays_not_intersecting)
{
    Ray ray1 = Ray(Point(-4, 0), Vector(0, -1));
    Ray ray2 = Ray(Point(4, 0), Vector(0, -1));

    Segment segment = Segment(Point(-1, 4), Point(1, 4));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(false, intersecting_segment.has_value());
}

// Test that the segment between the intersecting ray and the segment extreme is returned
// when one ray intersects, and the other would intersect an infinitely long segment
TEST(MultipleRaysTest, test_segment_intersect_one_ray_intersect_extreme1)
{
    Ray ray_intersecting          = Ray(Point(0, 0), Vector(0, 1));
    Ray ray_intersects_out_of_seg = Ray(Point(0, 0), Vector(-14, 1));

    Segment segment = Segment(Point(-1, 2), Point(1, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray_intersecting, ray_intersects_out_of_seg, segment);

    EXPECT_EQ(intersecting_segment.value(), Segment(Point(0, 2), Point(-1, 2)));
}
// Test that the segment between the intersecting ray and the segment extreme is returned
// when one ray intersects, and the other would intersect an infinitely long segment
TEST(MultipleRaysTest, test_segment_intersect_one_ray_intersect_extreme2)
{
    Ray ray_intersecting          = Ray(Point(0, 0), Vector(0, 1));
    Ray ray_intersects_out_of_seg = Ray(Point(0, 0), Vector(20, 2));

    Segment segment = Segment(Point(-1, 2), Point(1, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray_intersecting, ray_intersects_out_of_seg, segment);

    EXPECT_EQ(intersecting_segment.value(), Segment(Point(0, 2), Point(1, 2)));
}

TEST(MultipleRaysTest, test_segment_intersect_segment_enclosed_by_rays)
{
    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));
    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(intersecting_segment.value(), segment);
}

// Test that the function returns the segment when the rays enclose the segemnt
TEST(MultipleRaysTest, test_segment_enclosed_between_rays_is_enclosed)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(segment, enclosed_segment.value());
}

// Test that the function returns null when the rays only partially intersect the segment
TEST(MultipleRaysTest, test_segment_enclosed_between_rays_is_partially_enclosed)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(-0.2, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(0.2, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(std::nullopt, enclosed_segment);
}

// Test that the function returns null if the segment is only partially enclosed with 1
// real intersection
TEST(MultipleRaysTest,
     test_segment_enclosed_between_rays_is_partially_enclosed_one_real_intersection)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(-20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(std::nullopt, enclosed_segment);
}
