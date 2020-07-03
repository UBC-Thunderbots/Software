#include "software/new_geom/util/projection.h"

#include <gtest/gtest.h>

TEST(ProjectionTest, test_reduce_segments_collinear)
{
    Segment seg1 = Segment(Point(0, 1), Point(0, 2));
    Segment seg2 = Segment(Point(0, 1.5), Point(0, 2.5));
    Segment seg3 = Segment(Point(0, 3), Point(0, 4));
    Segment seg4 = Segment(Point(0, 1.2), Point(0, 1.5));

    Segment seg5 = Segment(Point(0, 6), Point(0, 9));
    Segment seg6 = Segment(Point(0, 6), Point(0, 12));

    Segment seg7 = Segment(Point(0, -2), Point(0, -5));

    std::vector<Segment> segs = {seg1, seg2, seg3, seg4, seg5, seg6, seg7};

    std::optional<std::vector<Segment>> reduced_segs =
        projectSegmentsOntoVector(segs, segs.front().toVector());

    EXPECT_EQ(Segment(Point(0, 1), Point(0, 2.5)), reduced_segs.value()[0]);
}

TEST(ProjectionTest, test_reduce_segments_perpendicular)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));


    std::vector<Segment> segs = {seg1, seg2, seg3};

    std::optional<std::vector<Segment>> reduced_segs =
        projectSegmentsOntoVector(segs, segs.front().toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 10);
}


TEST(ProjectionTest, test_reduce_segments_perpendicular_and_parallel)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));
    Segment seg4 = Segment(Point(0, 2), Point(0, 15));
    Segment seg5 = Segment(Point(0, 7), Point(9, 7));


    std::vector<Segment> segs = {seg1, seg2, seg3, seg4, seg5};

    std::optional<std::vector<Segment>> reduced_segs =
        projectSegmentsOntoVector(segs, seg1.toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 15);
    EXPECT_EQ(reduced_segs->front().getEnd().y(), 15);
}

TEST(ProjectionTest, test_project_circles_origin_inside_circle)
{
    Point reference = Point(0, 0);
    Circle circle   = Circle(Point(0, 0), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle}, reference);

    EXPECT_EQ(proj_segments.size(), 1);
    EXPECT_EQ(proj_segments.front().length(), 10);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().x(), 5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().x(), -5.0);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_one_circle)
{
    Point reference = Point(0, 0);
    Circle circle   = Circle(Point(0, 4), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle}, reference);

    EXPECT_EQ(proj_segments.size(), 1);
    EXPECT_NEAR(proj_segments.front().length(), 1.26, 0.01);
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), 0.63, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -0.63, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_multiple_circles)
{
    Point reference = Point(0, 0);
    Circle circle1  = Circle(Point(-1, 4), 0.5);
    Circle circle2  = Circle(Point(1, 4), 0.5);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle1, circle2}, reference);

    EXPECT_EQ(proj_segments.size(), 2);
    EXPECT_NEAR(proj_segments.front().length(), 1.30, 0.01);

    // Circle 1
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), -0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);


    // Circle 2
    EXPECT_NEAR(proj_segments.back().getSegStart().x(), 1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.back().getEnd().x(), 0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_project_circles_multiple_circles_one_has_zero_projection)
{
    Point reference = Point(0, 0);
    Circle circle1  = Circle(Point(-1, 4), 0.5);
    Circle circle2  = Circle(Point(1, 4), 0.5);
    Circle circle3  = Circle(Point(5, -5), 1);
    Segment segment = Segment(Point(5, 5), Point(-5, 5));

    std::vector<Segment> proj_segments =
        projectCirclesOntoSegment(segment, {circle1, circle2, circle3}, reference);

    EXPECT_EQ(proj_segments.size(), 2);
    EXPECT_NEAR(proj_segments.front().length(), 1.30, 0.01);

    // Circle 1
    EXPECT_NEAR(proj_segments.front().getSegStart().x(), -0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.front().getEnd().x(), -1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.front().getEnd().y(), 5.0);


    // Circle 2
    EXPECT_NEAR(proj_segments.back().getSegStart().x(), 1.92, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getSegStart().y(), 5.0);
    EXPECT_NEAR(proj_segments.back().getEnd().x(), 0.62, 0.01);
    EXPECT_DOUBLE_EQ(proj_segments.back().getEnd().y(), 5.0);
}

TEST(ProjectionTest, test_get_empty_space_between_segment_no_space)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-10, 0), Point(0, 0));
    Segment seg2 = Segment(Point(0, 0), Point(10, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 0);
}

TEST(ProjectionTest, test_get_empty_space_between_segment_2_blocks)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-8, 0), Point(0, 0));
    Segment seg2 = Segment(Point(0, 0), Point(8, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 2);

    EXPECT_EQ(open_segs[0].length(), 2);
    EXPECT_EQ(open_segs[1].length(), 2);
}

TEST(ProjectionTest, test_get_empty_space_between_segment_2_blocks_3_open)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-8, 0), Point(-2, 0));
    Segment seg2 = Segment(Point(2, 0), Point(8, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 3);

    EXPECT_EQ(open_segs[0].length(), 2);
    EXPECT_EQ(open_segs[1].length(), 4);
    EXPECT_EQ(open_segs[2].length(), 2);
}

// Test that function returns the larger segment when considering 2 redundant segments
TEST(ProjectionTest, test_segment_redundancy_segments_are_redundant)
{
    Segment segment1 = Segment(Point(-1, -1), Point(1, 1));

    Segment segment2 = Segment(Point(-2, -2), Point(2, 2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment2);
}

// Test that function returns the larger segment when considering 2 non-parallel segments
TEST(ProjectionTest, test_segment_redundancy_segments_are_not_parallel)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(1, 4), Point(1, -4));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment, std::nullopt);
}

// Test that function returns one of the segments if they are exactly the same
TEST(ProjectionTest, test_segment_redundancy_segments_are_the_same)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(2, 2), Point(1, -2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment1);
}

// Check the case where both rays intersect the segment only once (forming an intersection
// segment within the segment)
TEST(ProjectionTest, test_segment_intersect_with_existing_segment)
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
TEST(ProjectionTest, test_segment_intersect_both_rays_not_intersecting)
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
TEST(ProjectionTest, test_segment_intersect_one_ray_intersect_extreme1)
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
TEST(ProjectionTest, test_segment_intersect_one_ray_intersect_extreme2)
{
    Ray ray_intersecting          = Ray(Point(0, 0), Vector(0, 1));
    Ray ray_intersects_out_of_seg = Ray(Point(0, 0), Vector(20, 2));

    Segment segment = Segment(Point(-1, 2), Point(1, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray_intersecting, ray_intersects_out_of_seg, segment);

    EXPECT_EQ(intersecting_segment.value(), Segment(Point(0, 2), Point(1, 2)));
}

TEST(ProjectionTest, test_segment_intersect_segment_enclosed_by_rays)
{
    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));
    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(intersecting_segment.value(), segment);
}

// Test that the function returns the segment when the rays enclose the segemnt
TEST(ProjectionTest, test_segment_enclosed_between_rays_is_enclosed)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(segment, enclosed_segment.value());
}

// Test that the function returns null when the rays only partially intersect the segment
TEST(ProjectionTest, test_segment_enclosed_between_rays_is_partially_enclosed)
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
TEST(ProjectionTest,
     test_segment_enclosed_between_rays_is_partially_enclosed_one_real_intersection)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(-20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(std::nullopt, enclosed_segment);
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(ProjectionTest, test_merge_segment_partially_overlapping)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(5, 5));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), Segment(Point(-2, -2), Point(5, 5)));
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(ProjectionTest, test_merge_segment_redundant_segments)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(1, 1));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), segment1);
}

TEST(ProjectionTest, test_circle_tangent_rays)
{
    Point reference          = Point(0, 0);
    Circle circle            = Circle(Point(0, 1), 0.5);
    std::pair<Ray, Ray> test = getCircleTangentRaysWithReferenceOrigin(reference, circle);

    EXPECT_EQ(test.second.getStart(), reference);

    EXPECT_LT((test.first.toUnitVector() - Vector(0.5, 0.866025)).length(), 0.001);
    EXPECT_LT((test.second.toUnitVector() - Vector(-0.5, 0.866025)).length(), 0.001);
}
