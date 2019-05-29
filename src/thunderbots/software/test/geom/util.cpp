#include "geom/util.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "geom/angle.h"
#include "geom/point.h"

// Set this to 1 to enable debug output.
#define DEBUG 0

#if DEBUG
#define dbgout std::cout
#else
std::ostringstream dbgout;
#endif

TEST(GeomUtilTest, dist_point_rectangle_point_within)
{
    Point p(1, 2.1);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(0, dist(p, rect));
}


TEST(GeomUtilTest, dist_point_rectangle_point_below_rectangle)
{
    Point p(1, 1);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(1.0, dist(p, rect));
}

TEST(GeomUtilTest, dist_point_rectangle_point_above_rectangle)
{
    Point p(1, 5);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(1.0, dist(p, rect));
}

TEST(GeomUtilTest, dist_point_rectangle_point_to_left_of_rectangle)
{
    Point p(-1, 3);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(1.0, dist(p, rect));
}

TEST(GeomUtilTest, dist_point_rectangle_point_right_of_rectangle)
{
    Point p(3, 3);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(1.0, dist(p, rect));
}

TEST(GeomUtilTest, dist_point_rectangle_point_down_and_left_of_rectangle)
{
    Point p(-2.0, 0);
    Rectangle rect({0, 2}, {2, 4});

    EXPECT_DOUBLE_EQ(std::sqrt(8.0), dist(p, rect));
}

TEST(GeomUtilTest, dist_line_vector2)
{
    double calculated_val, expected_val;

    // case 1
    Line test1line(Point(0, 1), Point(0, 0));
    Point test1p(2, 0);
    calculated_val = dist(test1line, test1p);
    expected_val   = 2;
    EXPECT_EQ(expected_val, calculated_val);

    // case 2
    Line test2line(Point(2, 0), Point(0, 2));
    Point test2p(0, 0);
    calculated_val = dist(test2line, test2p);
    expected_val   = sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // case 3
    Line test3line(Point(0, 0), Point(0, 0));
    Point test3p(1, 0);
    calculated_val = dist(test3line, test3p);
    expected_val   = 1;
    EXPECT_EQ(expected_val, calculated_val);

    Line line(Point(1, -1), Point(5, -2));
    Point p(2, -3);

    EXPECT_NEAR(1.69775, dist(line, p), 1e-5);

    p = Point(2, 1);
    EXPECT_NEAR(2.18282, dist(line, p), 1e-5);

    p = Point(2, 0);
    EXPECT_NEAR(1.21268, dist(line, p), 1e-5);
}

TEST(GeomUtilTest, test_proj_len)
{
    double calculated_val, expected_val;
    // test case 1
    Point test1p1(0, 0);
    Point test1p2(4, 4);
    Point test1p3(4, 0);
    calculated_val = proj_len(Segment(test1p1, test1p2), test1p3);
    expected_val   = 2 * sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // test case 2
    Point test2p1(0, 0);
    Point test2p2(4, 0);
    Point test2p3(4, 4);
    calculated_val = proj_len(Segment(test2p1, test2p2), test2p3);
    expected_val   = 4;
    EXPECT_EQ(expected_val, calculated_val);

    // test case 3
    Point test3p1(0, 0);
    Point test3p2(4, 4);
    Point test3p3(-4, -4);
    calculated_val = proj_len(Segment(test3p1, test3p2), test3p3);
    expected_val   = -4 * sqrt(2);
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);

    // test case 4
    Point test4p1(0, 0);
    Point test4p2(4, 1);
    Point test4p3(-4, -4);
    calculated_val = proj_len(Segment(test4p1, test4p2), test4p3);
    expected_val   = -sqrt(32) * (cos((M_PI / 4.0f) - atan(1.0f / 4.0f)));
    EXPECT_DOUBLE_EQ(expected_val, calculated_val);
}

TEST(GeomUtilTest, test_contains_triangle_point)
{
    // this triangle lies in the first quatren of the field, we can rota
    Point p1(0, 0);
    Point p2((std::rand() % 100) / 100, 0);
    Point p3((std::rand() % 100) / 100, (std::rand() % 100) / 100);
    Point p((std::rand() % 100) / 100, (std::rand() % 100) / 100);
    bool expected_val = true;

    // i don't know what is going on here, this part seems to not work very well
    // so we'll just abuse cross products to see on which side of each side of
    // the triangle it's on
    if (((p2 - p1).cross(p - p1) > 0) != ((p2 - p1).cross(p3 - p1) > 0))
        expected_val = false;
    if (((p3 - p2).cross(p - p2) > 0) != ((p3 - p2).cross(p1 - p2) > 0))
        expected_val = false;
    if (((p1 - p3).cross(p - p3) > 0) != ((p1 - p3).cross(p2 - p3) > 0))
        expected_val = false;

    bool calculated_val = contains(triangle(p1, p2, p3), p);
    EXPECT_EQ(expected_val, calculated_val);
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
        Point v = Point::createFromAngle(
            Angle::ofDegrees(std::rand() % 360));  // should be random number here
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
        !intersects(triangle(test1p1, test1p2, test1p3),
                    Circle(test1c,
                           test1radius)));  // circle is tangent to triangle, no intersect

    Point test2p1(-10, 0);
    Point test2p2(10, 0);
    Point test2p3(0, 15);
    Point test2c(0, 5);
    double test2radius = 1;
    EXPECT_TRUE(intersects(
        triangle(test2p1, test2p2, test2p3),
        Circle(test2c,
               test2radius)));  // circle is completely inside triangle, intersect

    Point test3p1(-5, -5);
    Point test3p2(5, -5);
    Point test3p3(0, 0);
    Point test3c(0, 1);
    double test3radius = 1;
    EXPECT_TRUE(
        !intersects(triangle(test3p1, test3p2, test3p3),
                    Circle(test3c,
                           test3radius)));  // circle is tangent to vertice, no intersect

    Point test4p1(-8, -5);
    Point test4p2(0, 0);
    Point test4p3(-3, -2);
    Point test4c(5, 5);
    double test4radius = 2;
    EXPECT_TRUE(
        !intersects(triangle(test4p1, test4p2, test4p3), Circle(test4c, test4radius)));

    Point test5p1(-2, -2);
    Point test5p2(2, -2);
    Point test5p3(0, 1);
    Point test5c(0, -1);
    double test5radius = 1;
    EXPECT_TRUE(
        intersects(triangle(test5p1, test5p2, test5p3), Circle(test5c, test5radius)));
}

TEST(GeomUtilTest, test_angle_sweep_circles)
{
    std::vector<Point> obs;
    obs.clear();
    obs.push_back(Point(-9, 10));
    obs.push_back(Point(9, 10));

    std::optional<std::pair<Point, Angle>> testpair_opt =
        angleSweepCircles(Point(0, 0), Point(10, 10), Point(-10, 10), obs, 1.0);

    // We expect to get a result
    ASSERT_TRUE(testpair_opt);

    std::pair<Point, Angle> testpair = *testpair_opt;

    EXPECT_TRUE((testpair.first.norm() - Point(0, 1)).len() < 0.0001);
    EXPECT_NEAR(75.449, testpair.second.toDegrees(), 1e-4);

    obs.clear();
    obs.push_back(Point(-4, 6));
    obs.push_back(Point(6, 8));
    obs.push_back(Point(4, 10));

    testpair_opt =
        angleSweepCircles(Point(0, 0), Point(10, 10), Point(-10, 10), obs, 1.0);

    // We expect to get a result
    ASSERT_TRUE(testpair_opt);

    testpair = *testpair_opt;

    EXPECT_TRUE((testpair.first.norm() - Point(-0.0805897, 0.996747)).len() < 0.0001);
    EXPECT_NEAR(42.1928, testpair.second.toDegrees(), 1e-4);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_no_obstacles)
{
    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {1, 1}, {-1, 1}, {}, 0.1);

    ASSERT_EQ(1, result.size());
    EXPECT_EQ(Point(0, 1), result[0].first);
    EXPECT_EQ(90, result[0].second.toDegrees());
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacles_pos_y_to_neg_y)
{
    // Test with a single obstacle the is centered on the line segment that we are
    // sweeping over

    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {1, -1}, {1, 1}, {{1, 0}}, 0.01);

    ASSERT_EQ(2, result.size());

    std::sort(result.begin(), result.end(),
              [](auto pair1, auto pair2) { return pair1.second < pair2.second; });

    EXPECT_EQ(1, result[0].first.x());
    EXPECT_NEAR(-0.40, result[0].first.y(), 0.05);
    EXPECT_EQ(1, result[1].first.x());
    EXPECT_NEAR(0.40, result[1].first.y(), 0.05);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacles_neg_y_to_pos_y)
{
    // Test with a single obstacle the is centered on the line segment that we are
    // sweeping over

    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {1, 1}, {1, -1}, {{1, 0}}, 0.01);

    ASSERT_EQ(2, result.size());

    std::sort(result.begin(), result.end(),
              [](auto pair1, auto pair2) { return pair1.second < pair2.second; });

    EXPECT_EQ(1, result[0].first.x());
    EXPECT_NEAR(-0.40, result[0].first.y(), 0.05);
    EXPECT_EQ(1, result[1].first.x());
    EXPECT_NEAR(0.40, result[1].first.y(), 0.05);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacles_line_over_neg_x_axis)
{
    // Test with a single obstacle the is centered on the line segment that we are
    // sweeping over

    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {-1, 1}, {-1, -1}, {{-1, 0}}, 0.01);

    ASSERT_EQ(2, result.size());

    std::sort(result.begin(), result.end(),
              [](auto pair1, auto pair2) { return pair1.second < pair2.second; });

    EXPECT_EQ(-1, result[0].first.x());
    EXPECT_NEAR(0.40, result[0].first.y(), 0.05);
    EXPECT_EQ(-1, result[1].first.x());
    EXPECT_NEAR(-0.40, result[1].first.y(), 0.05);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacles_line_over_pos_y_axis)
{
    // Test with a single obstacle the is centered on the line segment that we are
    // sweeping over

    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {-1, 1}, {1, 1}, {{0, 1}}, 0.01);

    ASSERT_EQ(2, result.size());

    std::sort(result.begin(), result.end(),
              [](auto pair1, auto pair2) { return pair1.second < pair2.second; });

    EXPECT_EQ(1, result[0].first.y());
    EXPECT_NEAR(0.40, result[0].first.x(), 0.05);
    EXPECT_EQ(1, result[1].first.y());
    EXPECT_NEAR(-0.40, result[1].first.x(), 0.05);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacles_line_over_neg_y_axis)
{
    // Test with a single obstacle the is centered on the line segment that we are
    // sweeping over

    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({0, 0}, {-1, -1}, {1, -1}, {{0, -1}}, 0.01);

    ASSERT_EQ(2, result.size());

    std::sort(result.begin(), result.end(),
              [](auto pair1, auto pair2) { return pair1.second < pair2.second; });

    EXPECT_EQ(-1, result[0].first.y());
    EXPECT_NEAR(-0.40, result[0].first.x(), 0.05);
    EXPECT_EQ(-1, result[1].first.y());
    EXPECT_NEAR(0.40, result[1].first.x(), 0.05);
}

TEST(GeomUtilTest, test_angle_sweep_circles_all_single_obstacle_blocks_whole_range)
{
    // Test where there is no way to draw a line from the start point to the
    // target line segment that we are sweeping over because there is one obstacle in the
    // way
    std::vector<std::pair<Point, Angle>> result =
        angleSweepCirclesAll({-1, -0.5}, {-4.5, 0.5}, {-4.5, -0.5}, {{-1.2, -0.5}}, 0.09);

    ASSERT_EQ(0, result.size());
}

TEST(GeomUtilTest, test_angle_sweep_circles_all)
{
    std::vector<Point> obs;
    obs.clear();
    obs.push_back(Point(-9, 10));
    obs.push_back(Point(9, 10));

    std::vector<std::pair<Point, Angle>> testpairs =
        angleSweepCirclesAll(Point(0, 0), Point(10, 10), Point(-10, 10), obs, 1.0);

    obs.clear();
    obs.push_back(Point(-4, 6));
    obs.push_back(Point(6, 8));
    obs.push_back(Point(4, 10));

    testpairs =
        angleSweepCirclesAll(Point(0, 0), Point(10, 10), Point(-10, 10), obs, 1.0);

    // TODO: Add assert statement
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
        EXPECT_DOUBLE_EQ(1.0, (i - Point(0, 0)).len());
}

TEST(GeomUtilTest, test_closest_lineseg_point)
{
    Point l1(-1, 1);
    Point l2(1, 1);

    EXPECT_TRUE((closestPointOnSeg(Point(0, 2), l1, l2) - Point(0, 1)).len() < 0.00001);
    EXPECT_TRUE((closestPointOnSeg(Point(-2, 1.5), l1, l2) - Point(-1, 1)).len() <
                0.00001);

    l1 = Point(-2, 1);
    l2 = Point(1, 2);

    EXPECT_TRUE((closestPointOnSeg(Point(1, 0), l1, l2) - Point(0.4, 1.8)).len() <
                0.00001);
    EXPECT_TRUE((closestPointOnSeg(Point(-1.4, 1.2), l1, l2) - Point(-1.4, 1.2)).len() <
                0.00001);
}

TEST(GeomUtilTest, test_line_circle_intersect)
{
    std::vector<Point> intersections =
        lineCircleIntersect(Point(0, 0), 1.0, Point(0, 3), Point(1, 3));
    EXPECT_TRUE(intersections.size() == 0);

    intersections = lineCircleIntersect(Point(0, 0), 1.0, Point(-1, 1), Point(1, 1));
    EXPECT_TRUE(intersections.size() == 1);
    EXPECT_TRUE((intersections[0] - Point(0, 1)).len() < 0.00001);

    // i don't know which intersections will come in which order
    intersections = lineCircleIntersect(Point(0, 0), 1.0, Point(-1, -1), Point(1, 1));
    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE(
        (intersections[0] - Point(1.0 / sqrt(2.0), 1.0 / sqrt(2.0))).len() < 0.00001 ||
        (intersections[0] - Point(-1.0 / sqrt(2.0), -1.0 / sqrt(2.0))).len() < 0.00001);
    EXPECT_TRUE(
        (intersections[1] - Point(1.0 / sqrt(2.0), 1.0 / sqrt(2.0))).len() < 0.00001 ||
        (intersections[1] - Point(-1.0 / sqrt(2.0), -1.0 / sqrt(2.0))).len() < 0.00001);
}

TEST(GeomUtilTest, test_line_rect_intersect)
{
    std::vector<Point> intersections = lineRectIntersect(
        Rectangle(Point(-1, -1), Point(1, 1)), Point(-1, -2), Point(1, 2));

    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE((intersections[0] - Point(0.5, 1)).len() < 0.00001 ||
                (intersections[0] - Point(-0.5, -1)).len() < 0.00001);
    EXPECT_TRUE((intersections[1] - Point(0.5, 1)).len() < 0.00001 ||
                (intersections[1] - Point(-0.5, -1)).len() < 0.00001);

    intersections =
        lineRectIntersect(Rectangle(Point(0, 0), Point(1, 2)), Point(-1, 0), Point(4, 2));

    EXPECT_TRUE(intersections.size() == 2);
    EXPECT_TRUE((intersections[0] - Point(0, 0.4)).len() < 0.00001 ||
                (intersections[0] - Point(1, 0.8)).len() < 0.00001);
    EXPECT_TRUE((intersections[1] - Point(0, 0.4)).len() < 0.00001 ||
                (intersections[1] - Point(1, 0.8)).len() < 0.00001);

    intersections = lineRectIntersect(Rectangle(Point(-1, -1), Point(1, 1)), Point(0, 0),
                                      Point(1, 2));

    EXPECT_TRUE(intersections.size() == 1);
    EXPECT_TRUE((intersections[0] - Point(0.5, 1)).len() < 0.00001);

    intersections = lineRectIntersect(Rectangle(Point(-1, -1), Point(1, 1)),
                                      Point(-0.5, -0.5), Point(0.5, 0.5));

    EXPECT_TRUE(intersections.size() == 0);
}

TEST(GeomUtilTest, test_vector_rect_intersect)
{
    dbgout << "========= Enter vectorRectIntersect Test =========" << std::endl;
    Rectangle rect({1.0, 1.0}, {-1.0, -1.0});
    Point pr1(((std::rand() % 200) - 100) / 100.0, 1.0);
    Point pr2(((std::rand() % 200) - 100) / 100.0, -1.0);
    Point pr3(1.0, ((std::rand() % 200) - 100) / 100.0);
    Point pr4(-1.0, ((std::rand() % 200) - 100) / 100.0);
    Point pb(((std::rand() % 200) - 100) / 100.0, ((std::rand() % 200) - 100) / 100.0);
    Point pe1    = (pr1 - pb).norm() + pr1;
    Point pe2    = (pr2 - pb).norm() + pr2;
    Point pe3    = (pr3 - pb).norm() + pr3;
    Point pe4    = (pr4 - pb).norm() + pr4;
    Point found1 = vectorRectIntersect(rect, pb, pr1);
    Point found2 = vectorRectIntersect(rect, pb, pr2);
    Point found3 = vectorRectIntersect(rect, pb, pr3);
    Point found4 = vectorRectIntersect(rect, pb, pr4);

    // uncomment to print out some debugging info
    dbgout << " vectorA (" << pb.x() << ", " << pb.y() << ") " << std::endl;
    dbgout << " Intersect1 (" << pr1.x() << ", " << pr1.y() << ") "
           << " found1 (" << found1.x() << ", " << found1.y() << ") " << std::endl;
    dbgout << " Intersect2 (" << pr2.x() << ", " << pr2.y() << ") "
           << " found2 (" << found2.x() << ", " << found2.y() << ") " << std::endl;
    dbgout << " Intersect3 (" << pr3.x() << ", " << pr3.y() << ") "
           << " found3 (" << found3.x() << ", " << found3.y() << ") " << std::endl;
    dbgout << " Intersect4 (" << pr4.x() << ", " << pr4.y() << ") "
           << " found4 (" << found4.x() << ", " << found4.y() << ") " << std::endl;
    dbgout << " vectorB1 (" << pe1.x() << ", " << pe1.y() << ") " << std::endl;
    dbgout << " vectorB2 (" << pe2.x() << ", " << pe2.y() << ") " << std::endl;
    dbgout << " vectorB3 (" << pe3.x() << ", " << pe3.y() << ") " << std::endl;
    dbgout << " vectorB4 (" << pe4.x() << ", " << pe4.y() << ") " << std::endl;

    EXPECT_TRUE((found1 - pr1).len() < 0.001);
    EXPECT_TRUE((found2 - pr2).len() < 0.001);
    EXPECT_TRUE((found3 - pr3).len() < 0.001);
    EXPECT_TRUE((found4 - pr4).len() < 0.001);
}

TEST(GeomUtilTest, test_clip_point)
{
    Point rect1(-2, -1);
    Point rect2(2, 1);

    EXPECT_TRUE((clipPoint(Point(1, 1), rect1, rect2) - Point(1, 1)).len() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 1), rect1, rect2) - Point(2, 1)).len() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 2), rect1, rect2) - Point(2, 1)).len() < 0.00001);
}

TEST(GeomUtilTest, test_clip_point2)
{
    Rectangle r(Point(-2, -1), Point(2, 1));

    EXPECT_TRUE((clipPoint(Point(1, 1), r) - Point(1, 1)).len() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 1), r) - Point(2, 1)).len() < 0.00001);
    EXPECT_TRUE((clipPoint(Point(3, 2), r) - Point(2, 1)).len() < 0.00001);
}

TEST(GeomUtilTest, test_unique_line_intersect)
{
    EXPECT_TRUE(uniqueLineIntersects(Point(0, 0), Point(2, 2), Point(1, 0), Point(0, 1)));
    EXPECT_TRUE(
        !uniqueLineIntersects(Point(0, 0), Point(1, 1), Point(-1, 0), Point(0, 1)));
}

TEST(GeomUtilTest, test_line_intersect)
{
    dbgout << "========= Enter lineIntersection Test ========" << std::endl;

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

        // uncomment to print out some messages
        dbgout << "points are (" << a1.x() << ", " << a1.y() << ") ";
        dbgout << " (" << a2.x() << ", " << a2.y() << ") ";
        dbgout << " (" << b1.x() << ", " << b1.y() << ") ";
        dbgout << " (" << b2.x() << ", " << b2.y() << ") " << std::endl;
        dbgout << "expecting (" << expected.x() << ", " << expected.y() << ") "
               << std::endl;
        dbgout << "found (" << found.x() << ", " << found.y() << ") " << std::endl;

        EXPECT_TRUE((expected - found).len() < 0.0001);
    }
}

TEST(GeomUtilTest, test_seg_crosses_seg)
{
    dbgout << "========= Enter seg_crosses_seg Test ========" << std::endl;

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
        dbgout << "points are (" << a1.x() << ", " << a1.y() << ") ";
        dbgout << " (" << a2.x() << ", " << a2.y() << ") ";
        dbgout << " (" << b1.x() << ", " << b1.y() << ") ";
        dbgout << " (" << b2.x() << ", " << b2.y() << ") ";
        dbgout << " (" << i0.x() << ", " << i0.y() << ") ";
        dbgout << " a_over " << (a_over ? "true" : "false") << " b_over "
               << (b_over ? "true" : "false") << std::endl;
        dbgout << "expecting " << (expected ? "true" : "false") << " found "
               << (found ? "true" : "false") << std::endl;

        EXPECT_EQ(expected, found);
    }
}

TEST(GeomUtilTest, test_vector_crosses_seg)
{
    dbgout << "========= Enter vector_crosses_seg Test ========" << std::endl;

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

        Point a2 = a1 + (i0 - a1).norm();
        Point b2 =
            b1 + (i0 - b1) * (1 + (std::rand() % 100 / 100.0) *
                                      (expected ? 1 : -1));  // as a scaling factor for b2

        Vector ray_direction = (a2 - a1).norm();

        bool found = intersects(Ray(a1, ray_direction), Segment(b1, b2));

        // uncomment to print out some messages
        dbgout << "points are (" << a1.x() << ", " << a1.y() << ") ";
        dbgout << " (" << a2.x() << ", " << a2.y() << ") ";
        dbgout << " (" << b1.x() << ", " << b1.y() << ") ";
        dbgout << " (" << b2.x() << ", " << b2.y() << ") ";
        dbgout << " (" << i0.x() << ", " << i0.y() << ") ";
        dbgout << "expecting " << (expected ? "true" : "false") << " found "
               << (found ? "true" : "false") << std::endl;

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

        Point a2 = a1 - (i0 - a1).norm();
        Point b2 = b1 + (i0 - b1) * (1 + std::rand() % 100 / 100.0);  // as a scaling
        // factor for b2,
        // make sure it
        // is long enough

        Vector ray_direction = (a2 - a1).norm();

        bool found = intersects(Ray(a1, ray_direction), Segment(b1, b2));

        // uncomment to print out some messages
        dbgout << "points are (" << a1.x() << ", " << a1.y() << ") ";
        dbgout << " (" << a2.x() << ", " << a2.y() << ") ";
        dbgout << " (" << b1.x() << ", " << b1.y() << ") ";
        dbgout << " (" << b2.x() << ", " << b2.y() << ") ";
        dbgout << " (" << i0.x() << ", " << i0.y() << ") ";
        dbgout << "expecting " << (expected ? "true" : "false") << " found "
               << (found ? "true" : "false") << std::endl;

        EXPECT_EQ(expected, found);
    }
}

TEST(GeomUtilTest, test_reflect)
{
    Point ray0(2, 4);
    Point normal(-1, 1);
    Point reflected = reflect(ray0, normal);

    EXPECT_TRUE((reflected - Point(4, 2)).len() < 0.0001);
}

TEST(GeomUtilTest, test_reflect2)
{
    Point ray0(2, 4);
    Point line0(0, 0);
    Point line1(1, 1);
    Point reflected = reflect(line0, line1, ray0);

    EXPECT_TRUE((reflected - Point(4, 2)).len() < 0.0001);
}

TEST(GeomUtilTest, test_calc_block_cone)
{
    Point a(5, 10);
    Point b(-5, 10);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(0, sqrt(5))).len() < 0.00001);

    a = Point(0, 8);
    b = Point(4, 4);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(1, 1.0 + sqrt(2))).len() < 0.00001);

    a = Point(2, -4);
    b = Point(6, -2);

    EXPECT_TRUE((calcBlockCone(a, b, 1) - Point(1.9741, -1.71212)).len() < 0.00001);
}

TEST(GeomUtilTest, test_calc_block_cone2)
{
    Point a(5, 10);
    Point b(-5, 10);
    Point o(0, 0);

    EXPECT_TRUE((calcBlockCone(a, b, o, 1) - Point(0, sqrt(5))).len() < 0.00001);

    a = Point(6, 11);
    b = Point(-4, 11);
    o = Point(1, 1);

    EXPECT_TRUE((calcBlockCone(a, b, o, 1) - Point(0, sqrt(5)) - o).len() < 0.00001);

    a = Point(-2, 6);
    b = Point(2, 2);
    o = Point(-2, -2);

    EXPECT_TRUE((calcBlockCone(a, b, o, 1) - Point(1, 1.0 + sqrt(2)) - o).len() < 0.0001);
}

TEST(GeomUtilTest, test_calc_block_other_ray)
{
    // I don't know what the function is supposed to return, so I just set the
    // test value to the return value of the function for now.
    Point p(-0.301176, -1.24471);
    Point a = calcBlockOtherRay(Point(1, 0), Point(0.2, 1), Point(0.4, 0.1));

    EXPECT_TRUE((a - p).len() < 0.00001);
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
        (segmentNearLine(seg0, seg1, line0, line1) - Point(1.105263, 2.63158)).len() <
        0.0001);

    seg0 = Point(0, 3);
    seg1 = Point(0, 4);

    EXPECT_TRUE((segmentNearLine(seg0, seg1, line0, line1) - Point(0, 3)).len() < 0.001);
}

TEST(GeomUtilTest, test_intersection)
{
    Point a1(-1, 0);
    Point a2(4, 1);
    Point b1(0, -1);
    Point b2(1, 4);

    EXPECT_TRUE((intersection(a1, a2, b1, b2) - Point(0.25, 0.25)).len() < 0.0001);

    a2 = Point(4, 2);

    EXPECT_TRUE((intersection(a1, a2, b1, b2) - Point(0.30435, 0.52174)).len() < 0.0001);
}

// Test to ensure that intersects(Ray, Segment) does not use ray.getDirection() as a point
// along the ray (Should be ray.getRayStart() + ray.GetDirection())
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

    Vector v1 = Vector::createFromAngle(Angle::ofDegrees(-70));
    Vector v2 = Vector::createFromAngle(Angle::ofDegrees(-120));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_angle_over_pos_y_axis)
{
    // Two vectors that form an acute angle over the positive y-axis

    Vector v1 = Vector::createFromAngle(Angle::ofDegrees(70));
    Vector v2 = Vector::createFromAngle(Angle::ofDegrees(120));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_180_degrees)
{
    Vector v1 = Vector::createFromAngle(Angle::ofDegrees(-90));
    Vector v2 = Vector::createFromAngle(Angle::ofDegrees(90));

    EXPECT_DOUBLE_EQ(180, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_large_angle_over_neg_x_axis)
{
    Vector v1 = Vector::createFromAngle(Angle::ofDegrees(-95));
    Vector v2 = Vector::createFromAngle(Angle::ofDegrees(99));

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
    Point v1(1, 1);
    Point x2(2, 0);
    Point v2(-1, 1);

    EXPECT_DOUBLE_EQ(1.0, closestPointTime(x1, v1, x2, v2));

    x1 = Point(0, 0);
    v1 = Point(0, 0);
    x2 = Point(-1, 1);
    v2 = Point(1, 0);

    EXPECT_DOUBLE_EQ(1.0, closestPointTime(x1, v1, x2, v2));

    x1 = Point(0, 0);
    v1 = Point(1, 1);
    x2 = Point(6, 0);
    v2 = Point(-2, 2);

    EXPECT_DOUBLE_EQ(1.8, closestPointTime(x1, v1, x2, v2));
}

TEST(GeomUtilTest, test_dist_point_seg)
{
    Point a1(0, 0);
    Point b1(1, 0);

    EXPECT_DOUBLE_EQ(1.0, dist(Point(0, 1), Segment(a1, b1)));
    EXPECT_DOUBLE_EQ(1.0, dist(Point(2, 0), Segment(a1, b1)));
    EXPECT_DOUBLE_EQ(1.0, dist(Point(1, -1), Segment(a1, b1)));
    EXPECT_DOUBLE_EQ(1.0, dist(Point(-1, 0), Segment(a1, b1)));

    Point a2(5, 2);
    Point b2(2, 7);
    Point c2(6.5369, 7.2131);

    EXPECT_NEAR(4.0, dist(c2, Segment(a2, b2)), 1e-5);
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment intersect once
TEST(GeomUtilTest, test_ray_segment_intersecting)
{
    Ray ray         = Ray(Point(1, 1), Point(0.3, -0.2));
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
    Ray ray         = Ray(Point(0, 0), Point(0.0, 1));
    Segment segment = Segment(Point(1, 1.1), Point(10, 1.1));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1, std::nullopt);
    EXPECT_EQ(intersection2, std::nullopt);
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping)
{
    Ray ray         = Ray(Point(1, 1.1), Point(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getRayStart());
    EXPECT_EQ(intersection2.value(), segment.getEnd());
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping at segment end and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping_single_point_at_seg_end)
{
    Ray ray         = Ray(Point(1, 5), Point(0.0, 1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getRayStart());
    EXPECT_EQ(intersection2.value(), segment.getEnd());
}

// Test to see if raySegmentIntersection() returns the correct parameters when the ray and
// segment are overlapping at segment start and parallel
TEST(GeomUtilTest, test_ray_segment_overlapping_single_point_at_seg_start)
{
    Ray ray         = Ray(Point(1, 1), Point(0.0, -1));
    Segment segment = Segment(Point(1, 1), Point(1, 5));

    auto [intersection1, intersection2] = raySegmentIntersection(ray, segment);

    EXPECT_EQ(intersection1.value(), ray.getRayStart());
    EXPECT_EQ(intersection2.value(), segment.getSegStart());
}

// Test to see if the segment start and end are returned if the ray passes through both
TEST(GeomUtilTest, test_ray_segment_overlapping_passes_through_seg_start_and_end)
{
    Ray ray         = Ray(Point(1, 0), Point(0.0, 1));
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

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
