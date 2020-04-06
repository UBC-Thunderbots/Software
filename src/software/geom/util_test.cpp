#include "software/geom/util.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/triangle.h"
#include "software/test_util/test_util.h"
#include "software/time/timestamp.h"

TEST(GeomUtilTest, test_collinear)
{
    for (unsigned int i = 0; i < 10; ++i)
    {
        Vector v = Vector::createFromAngle(
            Angle::fromDegrees((std::rand() % 360)));  // should be random number here
        Point pointA((std::rand() % 100) / 100.0, (std::rand() % 100) / 100.0);
        Point pointB = pointA + v * (std::rand() % 100) / 100.0;
        Point pointC = pointA - v * (std::rand() % 100) / 100.0;
        bool val     = collinear(pointA, pointB, pointC);
        EXPECT_TRUE(val);
    }
}

TEST(GeomUtilTest, test_closest_lineseg_point)
{
    Point l1(-1, 1);
    Point l2(1, 1);

    EXPECT_TRUE((closestPointOnSeg(Point(0, 2), l1, l2) - Point(0, 1)).length() <
                0.00001);
    EXPECT_TRUE((closestPointOnSeg(Point(-2, 1.5), l1, l2) - Point(-1, 1)).length() <
                0.00001);

    l1 = Point(-2, 1);
    l2 = Point(1, 2);

    EXPECT_TRUE((closestPointOnSeg(Point(1, 0), l1, l2) - Point(0.4, 1.8)).length() <
                0.00001);
    EXPECT_TRUE(
        (closestPointOnSeg(Point(-1.4, 1.2), l1, l2) - Point(-1.4, 1.2)).length() <
        0.00001);
}

TEST(GeomUtilTest, test_line_circle_intersect)
{
    std::vector<Point> intersections =
        lineCircleIntersect(Point(0, 0), 1.0, Point(0, 3), Point(1, 3));
    EXPECT_TRUE(intersections.size() == 0);

    intersections = lineCircleIntersect(Point(0, 0), 1.0, Point(-1, 1), Point(1, 1));
    EXPECT_TRUE(intersections.size() == 1);
    EXPECT_TRUE((intersections[0] - Point(0, 1)).length() < 0.00001);

    // i don't know which intersections will come in which order
    intersections = lineCircleIntersect(Point(0, 0), 1.0, Point(-1, -1), Point(1, 1));
    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE((intersections[0] - Point(1.0 / sqrt(2.0), 1.0 / sqrt(2.0))).length() <
                    0.00001 ||
                (intersections[0] - Point(-1.0 / sqrt(2.0), -1.0 / sqrt(2.0))).length() <
                    0.00001);
    EXPECT_TRUE((intersections[1] - Point(1.0 / sqrt(2.0), 1.0 / sqrt(2.0))).length() <
                    0.00001 ||
                (intersections[1] - Point(-1.0 / sqrt(2.0), -1.0 / sqrt(2.0))).length() <
                    0.00001);
}

TEST(GeomUtilTest, test_calc_block_cone2)
{
    Point a(5, 10);
    Point b(-5, 10);
    Point o(0, 0);

    EXPECT_TRUE((calcBlockCone(a, b, o, 1) - Point(0, sqrt(5))).length() < 0.00001);

    a = Point(6, 11);
    b = Point(-4, 11);
    o = Point(1, 1);

    EXPECT_TRUE((Point(calcBlockCone(a, b, o, 1) - Point(0, sqrt(5))) - o).length() <
                0.00001);

    a = Point(-2, 6);
    b = Point(2, 2);
    o = Point(-2, -2);

    EXPECT_TRUE(
        (Point(calcBlockCone(a, b, o, 1) - Point(1, 1.0 + sqrt(2))) - o).length() <
        0.0001);
}

TEST(GeomUtilTest, test_offset_to_line)
{
    Point x0(1, -2);
    Point x1(5, -2);
    Point p(2, -3);

    EXPECT_NEAR(1, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 1);

    EXPECT_NEAR(3, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 0);

    EXPECT_NEAR(2, offsetToLine(x0, x1, p), 1e-5);
}

TEST(GeomUtilTest, test_acuteVertexAngle_angle_over_neg_y_axis)
{
    // Two vectors that form an acute angle over the negative y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((-120)));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_angle_over_pos_y_axis)
{
    // Two vectors that form an acute angle over the positive y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((120)));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_180_degrees)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-90)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((90)));

    EXPECT_DOUBLE_EQ(180, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_large_angle_over_neg_x_axis)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-95)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((99)));

    EXPECT_DOUBLE_EQ(166, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertex_angle_between_points)
{
    Point p1(2, 0.5);
    Point p2(1, -0.5);
    Point p3(1, 0.5);
    EXPECT_DOUBLE_EQ(45, acuteVertexAngle(p1, p2, p3).toDegrees());
}

// Check the case where both rays intersect the segment only once (forming an intersection
// segment within the segment)
TEST(GeomUtilTest, test_segment_intersect_with_existing_segment)
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
TEST(GeomUtilTest, test_segment_intersect_both_rays_not_intersecting)
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
TEST(GeomUtilTest, test_segment_intersect_one_ray_intersect_extreme1)
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
TEST(GeomUtilTest, test_segment_intersect_one_ray_intersect_extreme2)
{
    Ray ray_intersecting          = Ray(Point(0, 0), Vector(0, 1));
    Ray ray_intersects_out_of_seg = Ray(Point(0, 0), Vector(20, 2));

    Segment segment = Segment(Point(-1, 2), Point(1, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray_intersecting, ray_intersects_out_of_seg, segment);

    EXPECT_EQ(intersecting_segment.value(), Segment(Point(0, 2), Point(1, 2)));
}

TEST(GeomUtilTest, test_segment_intersect_segment_enclosed_by_rays)
{
    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));
    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    std::optional<Segment> intersecting_segment =
        getIntersectingSegment(ray1, ray2, segment);

    EXPECT_EQ(intersecting_segment.value(), segment);
}

// Test that the function returns the segment when the rays enclose the segemnt
TEST(GeomUtilTest, test_segment_enclosed_between_rays_is_enclosed)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(-20, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(segment, enclosed_segment.value());
}

// Test that the function returns null when the rays only partially intersect the segment
TEST(GeomUtilTest, test_segment_enclosed_between_rays_is_partially_enclosed)
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
TEST(GeomUtilTest,
     test_segment_enclosed_between_rays_is_partially_enclosed_one_real_intersection)
{
    Segment segment = Segment(Point(-2, 2), Point(2, 2));

    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));

    Ray ray2 = Ray(Point(0, 0), Vector(-20, 1));

    std::optional<Segment> enclosed_segment =
        segmentEnclosedBetweenRays(segment, ray1, ray2);

    EXPECT_EQ(std::nullopt, enclosed_segment);
}

// Test that function returns the larger segment when considering 2 redundant segments
TEST(GeomUtilTest, test_segment_redundancy_segments_are_redundant)
{
    Segment segment1 = Segment(Point(-1, -1), Point(1, 1));

    Segment segment2 = Segment(Point(-2, -2), Point(2, 2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment2);
}

// Test that function returns the larger segment when considering 2 non-parallel segments
TEST(GeomUtilTest, test_segment_redundancy_segments_are_not_parallel)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(1, 4), Point(1, -4));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment, std::nullopt);
}

// Test that function returns one of the segments if they are exactly the same
TEST(GeomUtilTest, test_segment_redundancy_segments_are_the_same)
{
    Segment segment1 = Segment(Point(2, 2), Point(1, -2));

    Segment segment2 = Segment(Point(2, 2), Point(1, -2));

    std::optional<Segment> largest_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    EXPECT_EQ(largest_segment.value(), segment1);
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(GeomUtilTest, test_merge_segment_partially_overlapping)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(5, 5));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), Segment(Point(-2, -2), Point(5, 5)));
}

// Test if segments are merged if they are parallel and only partially overlapping
TEST(GeomUtilTest, test_merge_segment_redundant_segments)
{
    Segment segment1 = Segment(Point(-2, -2), Point(2, 2));

    Segment segment2 = Segment(Point(-1, -1), Point(1, 1));

    std::optional<Segment> merged_segment =
        mergeOverlappingParallelSegments(segment1, segment2);
    EXPECT_EQ(merged_segment.value(), segment1);
}

TEST(GeomUtilTest, test_find_open_circles_no_points_in_rectangle)
{
    // Test finding the open circles in rectangle with no points
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Circle> empty_circles = findOpenCircles(rectangle, {});

    ASSERT_EQ(0, empty_circles.size());
}

TEST(GeomUtilTest, test_find_open_circles_one_point_in_rectangle)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));
    std::vector<Point> points = {Point(0.9, 0.9)};

    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(4, empty_circles.size());

    EXPECT_EQ(Point(-1, -1), empty_circles[0].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(1.9, 2)),
                     empty_circles[0].getRadius());

    EXPECT_EQ(Point(-1, 1), empty_circles[1].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[1].getRadius());

    EXPECT_EQ(Point(1, 1), empty_circles[2].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[2].getRadius());

    EXPECT_EQ(Point(1, -1), empty_circles[3].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[3].getRadius());
}

TEST(GeomUtilTest, test_find_open_circles_two_points_in_rectangle)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));
    std::vector<Point> points = {Point(0.9, 0.9), Point(-0.9, 0.9)};

    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    std::vector<std::pair<Point, double>> expected_origin_and_radii = {
        std::make_pair(Point(0, 1), std::sqrt(std::pow(0.1, 2) + std::pow(0.9, 2))),
        std::make_pair(Point(0, -1), std::sqrt(std::pow(1.9, 2) + std::pow(0.9, 2))),
        std::make_pair(Point(-1, -1), std::sqrt(std::pow(0.1, 2) + std::pow(1.9, 2))),
        std::make_pair(Point(-1, 1), std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2))),
        std::make_pair(Point(1, 1), std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2))),
        std::make_pair(Point(1, -1), std::sqrt(std::pow(0.1, 2) + std::pow(1.9, 2)))};

    ASSERT_EQ(6, empty_circles.size());
    for (auto expected_origin_and_radius : expected_origin_and_radii)
    {
        bool found             = false;
        Point expected_origin  = expected_origin_and_radius.first;
        double expected_radius = expected_origin_and_radius.second;

        for (const Circle &empty_circle : empty_circles)
        {
            if (expected_origin == empty_circle.getOrigin())
            {
                EXPECT_DOUBLE_EQ(expected_radius, empty_circle.getRadius());
                found = true;
            }
        }

        EXPECT_TRUE(found);
    }
}

TEST(GeomUtilTest, test_find_open_circle_three_in_rectangle)
{
    Rectangle rectangle = ::Test::TestUtil::createSSLDivBField().fieldLines();

    std::vector<Point> points         = {Point(-1, -1), Point(1, -1), Point(0, 1)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(8, empty_circles.size());

    // Calculated from Voronoi diagram
    EXPECT_EQ(-4.5, empty_circles[2].getOrigin().x());
    EXPECT_NEAR(2.09, empty_circles[2].getOrigin().y(), 0.05);
    EXPECT_NEAR(4.628, empty_circles[2].getRadius(), 0.005);

    EXPECT_EQ(4.5, empty_circles[3].getOrigin().x());
    EXPECT_NEAR(2.09, empty_circles[3].getOrigin().y(), 0.05);
    EXPECT_NEAR(4.628, empty_circles[3].getRadius(), 0.005);

    EXPECT_EQ(Point(0, -3), empty_circles[6].getOrigin());
    EXPECT_NEAR(2.236, empty_circles[6].getRadius(), 0.005);

    EXPECT_EQ(Point(0, -0.25), empty_circles[7].getOrigin());
    EXPECT_NEAR(1.25, empty_circles[7].getRadius(), 0.005);

    // Corner Points
    EXPECT_EQ(Point(-4.5, 3), empty_circles[0].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(4.5, 2) + std::pow(2, 2)),
                     empty_circles[0].getRadius());

    EXPECT_EQ(Point(4.5, 3), empty_circles[1].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(4.5, 2) + std::pow(2, 2)),
                     empty_circles[1].getRadius());

    EXPECT_EQ(Point(-4.5, -3), empty_circles[4].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(3.5, 2) + std::pow(2, 2)),
                     empty_circles[4].getRadius());

    EXPECT_EQ(Point(4.5, -3), empty_circles[5].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(3.5, 2) + std::pow(2, 2)),
                     empty_circles[5].getRadius());
}

TEST(GeomUtilTest, test_find_open_circle_points_outside_of_box_one_in_box)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Point> points         = {Point(-2, -1), Point(3, -2), Point(-1.1, -1.1),
                                 Point(0.9, 0.9)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(4, empty_circles.size());

    EXPECT_EQ(Point(-1, -1), empty_circles[0].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(1.9, 2)),
                     empty_circles[0].getRadius());

    EXPECT_EQ(Point(-1, 1), empty_circles[1].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[1].getRadius());

    EXPECT_EQ(Point(1, 1), empty_circles[2].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[2].getRadius());

    EXPECT_EQ(Point(1, -1), empty_circles[3].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[3].getRadius());
}

TEST(GeomUtilTest, test_find_open_circle_points_outside_of_box)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Point> points         = {Point(-2, -1), Point(3, -2), Point(-1.1, -1.1)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(0, empty_circles.size());
}

TEST(GeomUtilTest, test_find_closest_point_zero_points)
{
    std::vector<Point> test_points = {};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(std::nullopt, findClosestPoint(reference_point, test_points));
}

TEST(GeomUtilTest, test_find_closest_point_one_point)
{
    std::vector<Point> test_points = {Point(-2, -1)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[0], findClosestPoint(reference_point, test_points));
}

TEST(GeomUtilTest, test_find_closest_point_two_points)
{
    std::vector<Point> test_points = {Point(-2, -1), Point(0.7, 0.6)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[1], findClosestPoint(reference_point, test_points));
}

TEST(GeomUtilTest, test_find_closest_point_two_points_the_same)
{
    std::vector<Point> test_points = {Point(0.7, 0.6), Point(0.7, 0.6)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[0], findClosestPoint(reference_point, test_points));
}

TEST(GeomUtilTest, test_find_closest_point_many_points)
{
    std::vector<Point> test_points = {Point(0.7, 0.6), Point(0.8, 0.6), Point(-0.7, -0.6),
                                      Point(0.1, 0.2), Point(-1, -3.4)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[1], findClosestPoint(reference_point, test_points));
}

TEST(GeomUtilTest, test_reduce_segments_collinear)
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
        combineToParallelSegments(segs, segs.front().toVector());

    EXPECT_EQ(Segment(Point(0, 1), Point(0, 2.5)), reduced_segs.value()[0]);
}

TEST(GeomUtilTest, test_reduce_segments_perpendicular)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));


    std::vector<Segment> segs = {seg1, seg2, seg3};

    std::optional<std::vector<Segment>> reduced_segs =
        combineToParallelSegments(segs, segs.front().toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 10);
}


TEST(GeomUtilTest, test_reduce_segments_perpendicular_and_parallel)
{
    Segment seg1 = Segment(Point(0, 0), Point(0, 10));
    Segment seg2 = Segment(Point(0, 1), Point(1, 1));
    Segment seg3 = Segment(Point(-5, -5), Point(5, -5));
    Segment seg4 = Segment(Point(0, 2), Point(0, 15));
    Segment seg5 = Segment(Point(0, 7), Point(9, 7));


    std::vector<Segment> segs = {seg1, seg2, seg3, seg4, seg5};

    std::optional<std::vector<Segment>> reduced_segs =
        combineToParallelSegments(segs, seg1.toVector());

    EXPECT_EQ(reduced_segs->size(), 1);
    EXPECT_EQ(reduced_segs->front().length(), 15);
    EXPECT_EQ(reduced_segs->front().getEnd().y(), 15);
}

TEST(GeomUtilTest, test_circle_tangent_rays)
{
    Point reference          = Point(0, 0);
    Circle circle            = Circle(Point(0, 1), 0.5);
    std::pair<Ray, Ray> test = getCircleTangentRaysWithReferenceOrigin(reference, circle);

    EXPECT_EQ(test.second.getStart(), reference);

    EXPECT_LT((test.first.toUnitVector() - Vector(0.5, 0.866025)).length(), 0.001);
    EXPECT_LT((test.second.toUnitVector() - Vector(-0.5, 0.866025)).length(), 0.001);
}

TEST(GeomUtilTest, test_project_circles_origin_inside_circle)
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

TEST(GeomUtilTest, test_project_circles_one_circle)
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

TEST(GeomUtilTest, test_project_circles_multiple_circles)
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

TEST(GeomUtilTest, test_project_circles_multiple_circles_one_has_zero_projection)
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

TEST(GeomUtilTest, test_get_empty_space_between_segment_no_space)
{
    Segment reference = Segment(Point(-10, 0), Point(10, 0));

    Segment seg1 = Segment(Point(-10, 0), Point(0, 0));
    Segment seg2 = Segment(Point(0, 0), Point(10, 0));

    std::vector<Segment> segs = {seg1, seg2};

    std::vector<Segment> open_segs = getEmptySpaceWithinParentSegment(segs, reference);

    EXPECT_EQ(open_segs.size(), 0);
}
TEST(GeomUtilTest, test_get_empty_space_between_segment_2_blocks)
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
TEST(GeomUtilTest, test_get_empty_space_between_segment_2_blocks_3_open)
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
