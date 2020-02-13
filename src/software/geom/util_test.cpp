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

TEST(GeomUtilTest, test_proj_len)
{
    double calculated_val, expected_val;
    // test case 1
    Point test1p1(0, 0);
    Point test1p2(4, 4);
    Vector test1p3(4, 0);
    calculated_val = proj_length(Segment(test1p1, test1p2), test1p3);
    expected_val   = 2 * sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // test case 2
    Point test2p1(0, 0);
    Point test2p2(4, 0);
    Vector test2p3(4, 4);
    calculated_val = proj_length(Segment(test2p1, test2p2), test2p3);
    expected_val   = 4;
    EXPECT_EQ(expected_val, calculated_val);

    // test case 3
    Point test3p1(0, 0);
    Point test3p2(4, 4);
    Vector test3p3(-4, -4);
    calculated_val = proj_length(Segment(test3p1, test3p2), test3p3);
    expected_val   = -4 * sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // test case 4
    Point test4p1(0, 0);
    Point test4p2(4, 1);
    Vector test4p3(-4, -4);
    calculated_val = proj_length(Segment(test4p1, test4p2), test4p3);
    expected_val   = -sqrt(32) * (cos((M_PI / 4.0f) - atan(1.0f / 4.0f)));
    EXPECT_NEAR(expected_val, calculated_val, 0.00001);
}

TEST(GeomUtilTest, test_segment_contains_point_no_x_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(0, 1));

    Point point = Point(0, 0.5);

    EXPECT_EQ(contains(segment, point), true);
}

TEST(GeomUtilTest, test_segment_contains_point_no_y_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(1, 0));

    Point point = Point(0.5, 0);

    EXPECT_EQ(contains(segment, point), true);
}

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

TEST(GeomUtilTest, test_intersects_triangle_circle)
{
    Point test1p1(-5, 0);
    Point test1p2(5, 0);
    Point test1p3(2, 5);
    Point test1c(0, -1);
    double test1radius = 1;
    EXPECT_TRUE(
        !intersects(Triangle(test1p1, test1p2, test1p3),
                    Circle(test1c,
                           test1radius)));  // circle is tangent to triangle, no intersect

    Point test2p1(-10, 0);
    Point test2p2(10, 0);
    Point test2p3(0, 15);
    Point test2c(0, 5);
    double test2radius = 1;
    EXPECT_TRUE(intersects(
        Triangle(test2p1, test2p2, test2p3),
        Circle(test2c,
               test2radius)));  // circle is completely inside triangle, intersect

    Point test3p1(-5, -5);
    Point test3p2(5, -5);
    Point test3p3(0, 0);
    Point test3c(0, 1);
    double test3radius = 1;
    EXPECT_TRUE(
        !intersects(Triangle(test3p1, test3p2, test3p3),
                    Circle(test3c,
                           test3radius)));  // circle is tangent to vertice, no intersect

    Point test4p1(-8, -5);
    Point test4p2(0, 0);
    Point test4p3(-3, -2);
    Point test4c(5, 5);
    double test4radius = 2;
    EXPECT_TRUE(
        !intersects(Triangle(test4p1, test4p2, test4p3), Circle(test4c, test4radius)));

    Point test5p1(-2, -2);
    Point test5p2(2, -2);
    Point test5p3(0, 1);
    Point test5c(0, -1);
    double test5radius = 1;
    EXPECT_TRUE(
        intersects(Triangle(test5p1, test5p2, test5p3), Circle(test5c, test5radius)));
}

TEST(GeomUtilTest, test_point_in_rectangle)
{
    // Point in 1st quadrant, rectangle in the 3rd quadrant. Should fail!
    EXPECT_TRUE(!contains(Rectangle(Point(0, 0), Point(-2, -2)), Point(1, 1)));

    // Point in 3rd quadrant, rectangle in the 3rd quadrant. Pass!
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(-2, -2)), Point(-1, -1)));

    // Point is one of the corners of the rectangle. Pass
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(2, 2)), Point(2, 2)));

    // Point is on the edge of the rectangle. Pass
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(3, 3)), Point(0, 1)));

    // Point in the 1st quadrant, rectangle in the 1st quadrant. Pass
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(3, 3)), Point(1, 2)));

    // Point in the 2nd quadrant, rectangle in the 2nd quadrant. Point is off
    // above, Fail.
    EXPECT_TRUE(!contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-2, 5)));

    // Point in the 2nd quadrant, rectangle in the 4th quadrant. Point is off to
    // the left, Fail.
    EXPECT_TRUE(!contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-7, 2)));

    // Point in the 2nd quadrant, rectangle centered at origin. Point is off
    // above, Fail.
    EXPECT_TRUE(contains(Rectangle(Point(1, 1), Point(-1, -1)), Point(0.5, 0.5)));

    // Point in the 2nd quadrant, rectangle centered at origin. Point is off to
    // the left, Fail.
    EXPECT_TRUE(!contains(Rectangle(Point(1, 1), Point(-1, -1)), Point(2, 2)));
}

TEST(GeomUtilTest, test_circle_boundaries)
{
    std::vector<Point> test_circle = circleBoundaries(Point(0, 0), 1, 6);

    for (Point i : test_circle)
        EXPECT_DOUBLE_EQ(1.0, (i - Point(0, 0)).length());
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

TEST(GeomUtilTest, test_line_rect_intersect)
{
    std::vector<Point> intersections = lineRectIntersect(
        Rectangle(Point(-1, -1), Point(1, 1)), Point(-1, -2), Point(1, 2));

    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE((intersections[0] - Point(0.5, 1)).length() < 0.00001 ||
                (intersections[0] - Point(-0.5, -1)).length() < 0.00001);
    EXPECT_TRUE((intersections[1] - Point(0.5, 1)).length() < 0.00001 ||
                (intersections[1] - Point(-0.5, -1)).length() < 0.00001);

    intersections =
        lineRectIntersect(Rectangle(Point(0, 0), Point(1, 2)), Point(-1, 0), Point(4, 2));

    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE((intersections[0] - Point(0, 0.4)).length() < 0.00001 ||
                (intersections[0] - Point(1, 0.8)).length() < 0.00001);
    EXPECT_TRUE((intersections[1] - Point(0, 0.4)).length() < 0.00001 ||
                (intersections[1] - Point(1, 0.8)).length() < 0.00001);

    intersections = lineRectIntersect(Rectangle(Point(-1, -1), Point(1, 1)), Point(0, 0),
                                      Point(1, 2));

    EXPECT_TRUE(intersections.size() == 1);
    EXPECT_TRUE((intersections[0] - Point(0.5, 1)).length() < 0.00001);

    intersections = lineRectIntersect(Rectangle(Point(-1, -1), Point(1, 1)),
                                      Point(-0.5, -0.5), Point(0.5, 0.5));

    EXPECT_TRUE(intersections.size() == 0);
}

TEST(GeomUtilTest, test_vector_rect_intersect)
{
    Rectangle rect({1.0, 1.0}, {-1.0, -1.0});
    Point pr1(((std::rand() % 200) - 100) / 100.0, 1.0);
    Point pr2(((std::rand() % 200) - 100) / 100.0, -1.0);
    Point pr3(1.0, ((std::rand() % 200) - 100) / 100.0);
    Point pr4(-1.0, ((std::rand() % 200) - 100) / 100.0);
    Point pb(((std::rand() % 200) - 100) / 100.0, ((std::rand() % 200) - 100) / 100.0);
    Point found1 = vectorRectIntersect(rect, pb, pr1);
    Point found2 = vectorRectIntersect(rect, pb, pr2);
    Point found3 = vectorRectIntersect(rect, pb, pr3);
    Point found4 = vectorRectIntersect(rect, pb, pr4);


    EXPECT_TRUE((found1 - pr1).length() < 0.001);
    EXPECT_TRUE((found2 - pr2).length() < 0.001);
    EXPECT_TRUE((found3 - pr3).length() < 0.001);
    EXPECT_TRUE((found4 - pr4).length() < 0.001);
}

TEST(GeomUtilTest, test_clip_point)
{
    Point rect1(-2, -1);
    Point rect2(2, 1);

    EXPECT_TRUE((clipPoint(Point(1, 1), rect1, rect2) - Point(1, 1)).length() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 1), rect1, rect2) - Point(2, 1)).length() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 2), rect1, rect2) - Point(2, 1)).length() < 0.00001);
}

TEST(GeomUtilTest, test_clip_point2)
{
    Rectangle r(Point(-2, -1), Point(2, 1));

    EXPECT_TRUE((clipPoint(Point(1, 1), r) - Point(1, 1)).length() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 1), r) - Point(2, 1)).length() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 2), r) - Point(2, 1)).length() < 0.00001);
}

TEST(GeomUtilTest, test_unique_line_intersect)
{
    EXPECT_TRUE(uniqueLineIntersects(Point(0, 0), Point(2, 2), Point(1, 0), Point(0, 1)));
    EXPECT_TRUE(
        !uniqueLineIntersects(Point(0, 0), Point(1, 1), Point(-1, 0), Point(0, 1)));
}

TEST(GeomUtilTest, test_line_intersect)
{
    // should check for the the rare cases

    for (int i = 0; i < 10; i++)
    {
        // generate three random points
        Point a1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point b1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point expected(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);

        // We do not know what the  tolorance of the function is, but we
        // probabaly should check if segments overlap completely

        Point a2 = a1 + (expected - a1) * (1 + std::rand() % 200 / 100.0);
        Point b2 = b1 + (expected - b1) * (1 + std::rand() % 200 / 100.0);

        Point found = lineIntersection(a1, a2, b1, b2).value();


        EXPECT_TRUE((expected - found).length() < 0.0001);
    }
}

TEST(GeomUtilTest, test_close_parallel_segments_dont_intersect)
{
    // This is a test from a bug found
    Segment seg_1(Point(1.049, -1.049), Point(1.95, -1.049));
    Segment seg_2(Point(2, -1), Point(1, -1));

    EXPECT_FALSE(intersects(seg_1, seg_2));
}

TEST(GeomUtilTest, test_seg_crosses_seg)
{
    // should check for the the rare cases

    for (int i = 0; i < 10; i++)
    {
        // generate three random points
        Point a1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point b1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point i0(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);

        // We do not know what the  tolorance of the function is, but we
        // probabaly should check if segments overlap completely

        bool a_over = std::rand() % 2;
        bool b_over = std::rand() % 2;

        Point a2 = a1 + (i0 - a1) * (1 + std::rand() % 100 / 100.0 *
                                             (a_over ? 1 : -1));  // the last part
        // generate a number
        // either bigger or
        // smaller than 1
        Point b2 =
            b1 +
            (i0 - b1) * (1 + std::rand() % 100 / 100.0 *
                                 (b_over ? 1 : -1));  // as a scaling factor for a2 and b2

        bool expected = a_over && b_over;
        bool found    = intersects(Segment(a1, a2), Segment(b1, b2));

        // uncomment to print out some messages


        EXPECT_EQ(expected, found);
    }
}

TEST(GeomUtilTest, test_vector_crosses_seg)
{
    // should check for the the rare cases

    // case where vector faces segment but segment may/ may not be long enough
    for (int i = 0; i < 5; i++)
    {
        // generate three random points
        Point a1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point b1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point i0(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);

        // We do not know what the  tolerance of the function is, but we
        // probably should check if segments overlap completely

        bool expected = std::rand() % 2;

        Point a2 = a1 + (i0 - a1).normalize();
        Point b2 =
            b1 + (i0 - b1) * (1 + (std::rand() % 100 / 100.0) *
                                      (expected ? 1 : -1));  // as a scaling factor for b2

        Vector ray_direction = (a2 - a1).normalize();

        bool found = intersects(Ray(a1, ray_direction), Segment(b1, b2));


        EXPECT_EQ(expected, found);
    }

    // case where vector does not face segment
    for (int i = 0; i < 5; i++)
    {
        // generate three random points
        Point a1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point b1(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);
        Point i0(std::rand() % 200 / 100.0, std::rand() % 200 / 100.0);

        // We do not know what the  tolerance of the function is, but we
        // probably should check if segments overlap completely

        bool expected = false;

        Point a2 = a1 - ((i0 - a1).normalize());
        Point b2 = b1 + (i0 - b1) * (1 + std::rand() % 100 / 100.0);  // as a scaling
        // factor for b2,
        // make sure it
        // is long enough

        Vector ray_direction = (a2 - a1).normalize();

        bool found = intersects(Ray(a1, ray_direction), Segment(b1, b2));


        EXPECT_EQ(expected, found);
    }
}

TEST(GeomUtilTest, test_reflect)
{
    Vector ray0(2, 4);
    Vector normal(-1, 1);
    Vector reflected = reflect(ray0, normal);

    EXPECT_TRUE((reflected - Vector(4, 2)).length() < 0.0001);
}

TEST(GeomUtilTest, test_reflect2)
{
    Point ray0(2, 4);
    Point line0(0, 0);
    Point line1(1, 1);
    Point reflected = reflect(line0, line1, ray0);

    EXPECT_TRUE((reflected - Point(4, 2)).length() < 0.0001);
}

TEST(GeomUtilTest, test_calc_block_cone)
{
    Vector a(5, 10);
    Vector b(-5, 10);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(0, sqrt(5))).length() < 0.00001);

    a = Vector(0, 8);
    b = Vector(4, 4);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(1, 1.0 + sqrt(2))).length() < 0.00001);

    a = Vector(2, -4);
    b = Vector(6, -2);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(1.9741, -1.71212)).length() < 0.00001);
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

TEST(GeomUtilTest, test_calc_block_other_ray)
{
    // I don't know what the function is supposed to return, so I just set the
    // test value to the return value of the function for now.
    Point p(-0.301176, -1.24471);
    Vector a = calcBlockOtherRay(Point(1, 0), Point(0.2, 1), Point(0.4, 0.1));

    EXPECT_TRUE((Point(a) - p).length() < 0.00001);
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

TEST(GeomUtilTest, test_offset_along_line)
{
    Point x0(1, -1);
    Point x1(5, -2);
    Point p(2, 1);

    EXPECT_NEAR(0.485071, offsetAlongLine(x0, x1, p), 1e-5);

    p = Point(3, 1);

    EXPECT_NEAR(1.45521, offsetAlongLine(x0, x1, p), 1e-5);

    p = Point(2, -2);

    EXPECT_NEAR(1.21268, offsetAlongLine(x0, x1, p), 1e-5);
}

TEST(GeomUtilTest, test_segment_near_line)
{
    Point seg0(0, 3);
    Point seg1(3, 2);
    Point line0(-1, 0);
    Point line1(3, 5);

    EXPECT_TRUE(
        (segmentNearLine(seg0, seg1, line0, line1) - Point(1.105263, 2.63158)).length() <
        0.0001);

    seg0 = Point(0, 3);
    seg1 = Point(0, 4);

    EXPECT_TRUE((segmentNearLine(seg0, seg1, line0, line1) - Point(0, 3)).length() <
                0.001);
}

// Test to ensure that intersects(Ray, Segment) does not use ray.getDirection() as a point
// along the ray (Should be ray.getStart() + ray.GetDirection())
TEST(GeomUtilTest, test_ray_intersect_position_and_direction_intersect_not_just_direction)
{
    Segment segment = Segment(Point(-1, 1), Point(1, 1));

    Ray position_and_direction = Ray(Point(-2, 0), Vector(0, 1));
    Ray just_direction         = Ray(Point(0, 0), Vector(0, 1));

    EXPECT_EQ(intersects(position_and_direction, segment), false);
    EXPECT_EQ(intersects(just_direction, segment), true);
}

TEST(GeomUtilTest, test_line_intersection_segments_collinear_overlap)
{
    Segment seg1(Point(0, 0), Point(2, 2));
    Segment seg2(Point(1, 1), Point(3, 3));

    auto retval = lineIntersection(seg1, seg2);
    EXPECT_TRUE(std::find(retval.begin(), retval.end(), Point(1, 1)) != retval.end());
    EXPECT_TRUE(std::find(retval.begin(), retval.end(), Point(2, 2)) != retval.end());
}

TEST(GeomUtilTest, test_line_intersection_segments_collinear_no_overlap)
{
    Segment seg1(Point(0, 0), Point(1, 1));
    Segment seg2(Point(2, 2), Point(3, 3));

    auto retval = lineIntersection(seg1, seg2);
    EXPECT_TRUE(std::find(retval.begin(), retval.end(), Point(1, 1)) == retval.end());
    EXPECT_TRUE(std::find(retval.begin(), retval.end(), Point(2, 2)) == retval.end());
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

TEST(GeomUtilTest, test_closest_point_time)
{
    Point x1(0, 0);
    Vector v1(1, 1);
    Point x2(2, 0);
    Vector v2(-1, 1);

    EXPECT_DOUBLE_EQ(1.0, closestPointTime(x1, v1, x2, v2));

    x1 = Point(0, 0);
    v1 = Vector(0, 0);
    x2 = Point(-1, 1);
    v2 = Vector(1, 0);

    EXPECT_DOUBLE_EQ(1.0, closestPointTime(x1, v1, x2, v2));

    x1 = Point(0, 0);
    v1 = Vector(1, 1);
    x2 = Point(6, 0);
    v2 = Vector(-2, 2);

    EXPECT_DOUBLE_EQ(1.8, closestPointTime(x1, v1, x2, v2));
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment intersect once
TEST(GeomUtilTest, test_ray_segment_intersecting)
{
    Ray ray         = Ray(Point(1, 1), Vector(0.3, -0.2));
    Segment segment = Segment(Point(-2, -2), Point(10, -2));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_DOUBLE_EQ(intersection1.value().x(), 5.5);
    EXPECT_DOUBLE_EQ(intersection1.value().y(), -2);
    EXPECT_EQ(intersection2, std::nullopt);
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment don't intersect
TEST(GeomUtilTest, test_ray_segment_non_intersecting)
{
    Ray ray         = Ray(Point(0, 0), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1.1), Point(10, 1.1));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1, std::nullopt);
    EXPECT_EQ(intersection2, std::nullopt);
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping)
{
    Ray ray         = Ray(Point(1, 1.1), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getStart());
    EXPECT_EQ(intersection2.value(), segment.getEnd());
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping at segment end and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping_single_point_at_seg_end)
{
    Ray ray         = Ray(Point(1, 5), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getStart());
    EXPECT_EQ(intersection2.value(), segment.getEnd());
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping at segment start and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping_single_point_at_seg_start)
{
    Ray ray         = Ray(Point(1, 1), Vector(0.0, -1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getStart());
    EXPECT_EQ(intersection2.value(), segment.getSegStart());
}

// Test to see if the segment start and end are returned if the ray passes through both
TEST(GeomUtilTest, test_ray_segment_overlapping_passes_through_seg_start_and_end)
{
    Ray ray         = Ray(Point(1, 0), Vector(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), segment.getSegStart());
    EXPECT_EQ(intersection2.value(), segment.getEnd());
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

// Test that a correct intersection point is returned for 2 rays that intersect once
TEST(GeomUtilTest, test_intersect_ray_ray_do_intersect_once)
{
    // Ray at origin pointing upwards
    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));
    // Ray up and to the right that points right
    Ray ray2 = Ray(Point(-1, 1), Vector(1, 0));

    std::optional<Point> intersection = getRayIntersection(ray1, ray2);

    EXPECT_EQ(intersection.value(), Point(0, 1));
}

// Test that an intersection is not returned if the opposite direction of the rays
// intersect
TEST(GeomUtilTest, test_intersect_ray_ray_reverse_direction_intersects)
{
    // Ray positioned at origin pointing down
    Ray ray1 = Ray(Point(0, 0), Vector(0, -1));

    Ray ray2 = Ray(Point(-1, 1), Vector(-1, 0));

    std::optional<Point> intersection = getRayIntersection(ray1, ray2);

    EXPECT_EQ(intersection, std::nullopt);
}

// Test that an intersection is not returned if the Rays are overlapping
TEST(GeomUtilTest, test_intersect_ray_ray_overlapping)
{
    // Ray positioned at origin pointing up
    Ray ray1 = Ray(Point(0, 0), Vector(0, 1));
    Ray ray2 = Ray(Point(0, 1), Vector(0, 1));

    std::optional<Point> intersection = getRayIntersection(ray1, ray2);

    EXPECT_EQ(intersection, std::nullopt);
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

// Test to see if the 1 is returned when the point exists within the rectangle
TEST(GeomUtilTest, test_binary_trespass_point_is_trespassing_in_rectangle)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(0, 0)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(0.5, 0.5)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(-0.5, -0.5)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(0.5, -0.5)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(-0.5, 0.5)));
}

// Test to see if the 1 is returned when the point exists on the boundries of the
// rectangle
TEST(GeomUtilTest, test_binary_trespass_point_is_on_rectangle_boundry)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(-1, -1)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(1, 1)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(-1, 1)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(1, -1)));
    EXPECT_EQ(1, calcBinaryTrespassScore(rectangle, Point(-1, 0.5)));
}

// Test to see if the 0 is returned when the point exists outside of the rectangle
TEST(GeomUtilTest, test_binary_trespass_point_is_outside_rectangle)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(0, calcBinaryTrespassScore(rectangle, Point(-1, -2)));
    EXPECT_EQ(0, calcBinaryTrespassScore(rectangle, Point(2, 1)));
    EXPECT_EQ(0, calcBinaryTrespassScore(rectangle, Point(-1, 3)));
    EXPECT_EQ(0, calcBinaryTrespassScore(rectangle, Point(5, -0.2)));
    EXPECT_EQ(0, calcBinaryTrespassScore(rectangle, Point(-4, 5)));
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

    ASSERT_EQ(6, empty_circles.size());

    EXPECT_EQ(Point(0, 1), empty_circles[0].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.9, 2)),
                     empty_circles[0].getRadius());

    EXPECT_EQ(Point(0, -1), empty_circles[1].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.9, 2)),
                     empty_circles[1].getRadius());

    EXPECT_EQ(Point(-1, -1), empty_circles[2].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(1.9, 2)),
                     empty_circles[2].getRadius());

    EXPECT_EQ(Point(-1, 1), empty_circles[3].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[3].getRadius());

    EXPECT_EQ(Point(1, 1), empty_circles[4].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[4].getRadius());

    EXPECT_EQ(Point(1, -1), empty_circles[5].getOrigin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(1.9, 2)),
                     empty_circles[5].getRadius());
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

TEST(GeomUtilTest, test_ray_rectangle_intersection_no_intersection)
{
    Ray ray(Point(5, 5), Vector(1, 1));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::pair<std::optional<Point>, std::optional<Point>> expected_result =
        std::make_pair(std::nullopt, std::nullopt);
    auto result = rayRectangleIntersection(ray, rectangle);

    EXPECT_EQ(result, expected_result);
}

TEST(GeomUtilTest,
     test_ray_rectangle_intersection_ray_start_inside_rectangle_intersects_side)
{
    Ray ray(Point(0, 0), Vector(1, 0));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::pair<std::optional<Point>, std::optional<Point>> expected_result =
        std::make_pair(Point(1, 0), std::nullopt);
    auto result = rayRectangleIntersection(ray, rectangle);

    EXPECT_EQ(result, expected_result);
}

TEST(GeomUtilTest,
     test_ray_rectangle_intersection_ray_start_inside_rectangle_intersects_corner)
{
    Ray ray(Point(0, 0), Vector(1, 1));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::pair<std::optional<Point>, std::optional<Point>> expected_result =
        std::make_pair(Point(1, 1), std::nullopt);
    auto result = rayRectangleIntersection(ray, rectangle);

    EXPECT_EQ(result, expected_result);
}

TEST(GeomUtilTest, test_ray_rectangle_intersection_ray_overlaps_rectangle_segment)
{
    Ray ray(Point(-2, -1), Vector(1, 0));
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    std::pair<std::optional<Point>, std::optional<Point>> expected_result =
        std::make_pair(Point(-1, -1), Point(1, -1));
    auto result = rayRectangleIntersection(ray, rectangle);

    EXPECT_EQ(result, expected_result);
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
