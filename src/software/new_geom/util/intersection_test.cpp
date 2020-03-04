#include "software/new_geom/util/intersection.h"

#include <gtest/gtest.h>

#include "software/new_geom/rectangle.h"

TEST(SegmentSegmentIntersectionsTest, test_segments_single_point_intersection)
{
    Segment a                        = Segment(Point(2, 2), Point(-2, -2));
    Segment b                        = Segment(Point(-2, 2), Point(2, -2));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(intersections[0], Point());
}

TEST(SegmentSegmentIntersectionsTest, test_segments_no_intersection)
{
    Segment a                        = Segment(Point(5, 2), Point(-2, -2));
    Segment b                        = Segment(Point(-3, -3), Point(-9, 10));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 0);
}

TEST(SegmentSegmentIntersectionsTest, test_segments_overlapping)
{
    Segment a                        = Segment(Point(0, 3), Point(5, 3));
    Segment b                        = Segment(Point(1, 3), Point(4, 3));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 2);
    EXPECT_EQ(intersections[0], Point(1, 3));
    EXPECT_EQ(intersections[1], Point(4, 3));
}

TEST(RectangleSegmentIntersectionsTest, test_no_intersections_segment_inside)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(1, 1), Point(3, 4));
    std::unordered_set<Point> intersections = intersection(r, s);
    EXPECT_TRUE(intersections.empty());
}

TEST(RectangleSegmentIntersectionsTest, test_no_intersections_segment_outside)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(-2, 2.5), Point(-4, 4));
    std::unordered_set<Point> intersections = intersection(r, s);
    EXPECT_TRUE(intersections.empty());
}

TEST(RectangleSegmentIntersectionsTest, test_one_intersection)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(0, 2.5), Point(3, 4));
    std::unordered_set<Point> intersections = intersection(r, s);
    std::unordered_set<Point> expected      = {Point(0, 2.5)};
    EXPECT_EQ(intersections, expected);
}

TEST(RectangleSegmentIntersectionsTest, test_two_intersections)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(0, 2.5), Point(5, 2.5));
    std::unordered_set<Point> intersections = intersection(r, s);
    std::unordered_set<Point> expected      = {Point(0, 2.5), Point(5, 2.5)};
    EXPECT_EQ(intersections, expected);
}

TEST(RectangleSegmentIntersectionsTest, test_three_intersections)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(0, 2.5), Point(0, 5));
    std::unordered_set<Point> intersections = intersection(r, s);
    std::unordered_set<Point> expected      = {Point(0, 2.5), Point(0, 5)};
    EXPECT_EQ(intersections, expected);
}

TEST(RectangleSegmentIntersectionsTest, test_four_intersections)
{
    Rectangle r                             = Rectangle(Point(), Point(5, 5));
    Segment s                               = Segment(Point(), Point(5, 5));
    std::unordered_set<Point> intersections = intersection(r, s);
    std::unordered_set<Point> expected      = {Point(), Point(5, 5)};
    EXPECT_EQ(intersections, expected);
}

TEST(RaySegmentIntersectionsTest, test_ray_segment_intersecting)
{
    Ray ray         = Ray(Point(1, 1), Vector(0.3, -0.2));
    Segment segment = Segment(Point(-2, -2), Point(10, -2));

    auto intersections = intersection(ray, segment);

    EXPECT_EQ(intersections.size(), 1);
    EXPECT_DOUBLE_EQ(intersections[0].x(), 5.5);
    EXPECT_DOUBLE_EQ(intersections[0].y(), -2);
}

TEST(RaySegmentIntersectionsTest, test_ray_segment_non_intersecting)
{
    Ray ray         = Ray(Point(0, 0), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1.1), Point(10, 1.1));

    auto intersections = intersection(ray, segment);
    EXPECT_TRUE(intersections.empty());
}

TEST(RaySegmentIntersectionsTest, test_ray_start_contained_within_segment)
{
    Ray ray         = Ray(Point(1, 1.1), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto intersections = intersection(ray, segment);

    EXPECT_EQ(intersections[0], ray.getStart());
    EXPECT_EQ(intersections[1], segment.getEnd());
}

TEST(RaySegmentIntersectionsTest, test_ray_overlapping_single_point_at_seg_end)
{
    Ray ray         = Ray(Point(1, 5), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto intersections = intersection(ray, segment);

    EXPECT_EQ(intersections[0], ray.getStart());
    EXPECT_EQ(intersections[1], segment.getEnd());
}

TEST(RaySegmentIntersectionsTest, test_ray_overlapping_single_point_at_seg_start)
{
    Ray ray         = Ray(Point(1, 1), Vector(0.0, -1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto intersections = intersection(ray, segment);

    EXPECT_EQ(intersections[0], ray.getStart());
    EXPECT_EQ(intersections[1], segment.getSegStart());
}

TEST(RaySegmentIntersectionsTest, test_ray_passes_through_seg_start_and_end)
{
    Ray ray         = Ray(Point(1, 0), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto intersections = intersection(ray, segment);

    EXPECT_EQ(intersections[0], segment.getSegStart());
    EXPECT_EQ(intersections[1], segment.getEnd());
}

TEST(LineLineIntersectionTest, test_intersection)
{
    Line l_1 = Line(Point(0, -3), Point(3, 6));
    Line l_2 = Line(Point(0, 4), Point(1, 8));

    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_EQ(point_of_intersection.value(), Point(-7, -24));
}

TEST(LineLineIntersectionTest, test_overlapping_not_intersecting)
{
    Line l_1 = Line(Point(0, -3), Point(3, 6));
    Line l_2 = Line(Point(1, 0), Point(2, 3));

    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_FALSE(point_of_intersection.has_value());
}

TEST(LineLineIntersectionTest, test_other_intersection)
{
    Line l_1                   = Line(Point(-1, -1), Point(5, -1));
    Line l_2                   = Line(Point(0, 2), Point(10, 8));
    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_EQ(point_of_intersection.value(), Point(-5, -1));
}

TEST(LineLineIntersectionTest, test_vertical_horizontal_lines_cross)
{
    Line l_1                   = Line(Point(), Point(5, 0));
    Line l_2                   = Line(Point(), Point(0, 5));
    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_EQ(point_of_intersection.value(), Point());
}

TEST(LineLineIntersectionTest, test_no_intersection)
{
    Line l_1 = Line(Point(0, -3), Point(3, 6));
    Line l_2 = Line(Point(2, -3), Point(5, 6));

    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_FALSE(point_of_intersection.has_value());
}

TEST(LineLineIntersectionTest, test_parallel_lines_no_intersection)
{
    Line l_1                   = Line(Point(), Point(5, 0));
    Line l_2                   = Line(Point(0, 2), Point(10, 2));
    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_FALSE(point_of_intersection.has_value());
}

TEST(LineLineIntersectionTest, test_no_intersection_overlapping_lines)
{
    Line l_1                   = Line(Point(), Point(10, 15));
    Line l_2                   = Line(Point(10, 15), Point(20, 30));
    auto point_of_intersection = intersection(l_1, l_2);
    EXPECT_FALSE(point_of_intersection.has_value());
}

TEST(RayRectangleIntersectionsTest, test_ray_rectangle_intersection_no_intersection)
{
    Ray ray             = Ray(Point(5, 5), Vector(1, 1));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    auto intersections = intersection(rectangle, ray);

    EXPECT_TRUE(intersections.empty());
}

TEST(RayRectangleIntersectionsTest, test_ray_start_inside_rectangle_intersects_side)
{
    Ray ray             = Ray(Point(0, 0), Vector(1, 0));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::unordered_set<Point> expected_result = {Point(1, 0)};
    auto intersections                        = intersection(rectangle, ray);

    EXPECT_EQ(intersections, expected_result);
}

TEST(RayRectangleIntersectionsTest, test_ray_outside_intersects_two_segments)
{
    Ray ray(Point(-2, 0), Vector(1, 0));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::unordered_set<Point> expected_result = {Point(-1, 0), Point(1, 0)};
    auto intersections                        = intersection(rectangle, ray);

    EXPECT_EQ(intersections, expected_result);
}

TEST(RayRectangleIntersectionsTest, test_ray_overlaps_rectangle_segment)
{
    Ray ray(Point(-2, -1), Vector(1, 0));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::unordered_set<Point> expected_result = {Point(-1, -1), Point(1, -1)};
    auto intersections                        = intersection(rectangle, ray);

    EXPECT_EQ(intersections, expected_result);
}

TEST(RayRayIntersectionTest, test_rays_intersect)
{
    // Ray at origin pointing upwards
    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));
    // Ray up and to the right that points right
    Ray ray2 = Ray(Point(-1, 1), Vector(1, 0));

    std::optional<Point> point_of_intersection = intersection(ray1, ray2);

    EXPECT_EQ(point_of_intersection.value(), Point(0, 1));
}

TEST(RayRayIntersectionTest, test_rays_reverse_direction_intersects_no_intersection)
{
    // Ray positioned at origin pointing down
    Ray ray1 = Ray(Point(0, 0), Vector(0, -1));

    // Ray positioned NW of ray1 pointing left
    Ray ray2 = Ray(Point(-1, 1), Vector(-1, 0));

    std::optional<Point> point_of_intersection = intersection(ray1, ray2);

    EXPECT_EQ(point_of_intersection, std::nullopt);
}

TEST(RayRayIntersectionTest, test_rays_overlapping_no_intersection)
{
    // Ray positioned at origin pointing up
    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));
    Ray ray2 = Ray(Point(0, 1), Vector(0, 1));

    std::optional<Point> point_of_intersection = intersection(ray1, ray2);

    EXPECT_EQ(point_of_intersection, std::nullopt);
}
