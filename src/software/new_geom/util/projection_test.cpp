#include "software/new_geom/util/projection.h"

#include <gtest/gtest.h>

#include "software/new_geom/util/projection_impl.h"
#include "software/test_util/test_util.h"

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

TEST(ProjectionImplTest, test_circle_tangent_points)
{
    Point p(2, 3);
    Circle c(Point(-1, 0), 2);
    auto [p1, p2] = getCircleTangentPoints(p, c);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(p1, Point(-1.58, 1.91), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(p2, Point(0.91, -0.58), 0.01));
}

TEST(ProjectionImplTest, test_circle_tangent_rays)
{
    Point p(2, 3.5);
    Circle c(Point(-1, 0), 2);
    auto [r1, r2] = getCircleTangentRaysWithReferenceOrigin(p, c);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r1.getStart(), p, 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r2.getStart(), p, 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r1.getDirection(),
                                               (Point(-1.58, 1.91) - p).orientation(),
                                               Angle::fromRadians(0.01)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r2.getDirection(),
                                               (Point(0.91, -0.58) - p).orientation(),
                                               Angle::fromRadians(0.01)));
}

// Check the case where both rays intersect the segment only once (forming an intersection
// segment within the segment)
TEST(ProjectionImplTest, test_segment_intersect_with_existing_segment)
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
TEST(ProjectionImplTest, test_segment_intersect_both_rays_not_intersecting)
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
TEST(ProjectionImplTest, test_segment_intersect_one_ray_intersect_extreme1)
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
TEST(ProjectionImplTest, test_segment_intersect_one_ray_intersect_extreme2)
{
    Ray ray_intersecting          = Ray(Point(0, 0), Vector(0, 1));
    Ray ray_intersects_out_of_seg = Ray(Point(0, 0), Vector(20, 2));

    Segment segment = Segment(Point(-1, 2), Point(1, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray_intersecting, ray_intersects_out_of_seg, segment);

    EXPECT_EQ(intersecting_segment.value(), Segment(Point(0, 2), Point(1, 2)));
}

TEST(ProjectionImplTest, test_segment_intersect_segment_enclosed_by_rays)
{
    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));
    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(intersecting_segment.value(), segment);
}

// Test that the function returns the segment when the rays enclose the segemnt
TEST(ProjectionImplTest, test_segment_enclosed_between_rays_is_enclosed)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(segment, enclosed_segment.value());
}

// Test that the function returns null when the rays only partially intersect the segment
TEST(ProjectionImplTest, test_segment_enclosed_between_rays_is_partially_enclosed)
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
TEST(ProjectionImplTest,
     test_segment_enclosed_between_rays_is_partially_enclosed_one_real_intersection)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(-20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(std::nullopt, enclosed_segment);
}
