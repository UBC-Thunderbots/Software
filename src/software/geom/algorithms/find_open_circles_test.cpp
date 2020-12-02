#include "software/geom/algorithms/find_open_circles.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(FindOpenCirclesTest, test_find_open_circles_no_points_in_rectangle)
{
    // Test finding the open circles in rectangle with no points
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Circle> empty_circles = findOpenCircles(rectangle, {});

    ASSERT_EQ(0, empty_circles.size());
}

TEST(FindOpenCirclesTest, test_find_open_circles_one_point_in_rectangle)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));
    std::vector<Point> points = {Point(0.9, 0.9)};

    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(4, empty_circles.size());

    EXPECT_EQ(Point(-1, -1), empty_circles[0].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(1.9, 2)),
                     empty_circles[0].radius());

    EXPECT_EQ(Point(-1, 1), empty_circles[1].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[1].radius());

    EXPECT_EQ(Point(1, 1), empty_circles[2].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[2].radius());

    EXPECT_EQ(Point(1, -1), empty_circles[3].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[3].radius());
}

TEST(FindOpenCirclesTest, test_find_open_circles_two_points_in_rectangle)
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
            if (expected_origin == empty_circle.origin())
            {
                EXPECT_DOUBLE_EQ(expected_radius, empty_circle.radius());
                found = true;
            }
        }

        EXPECT_TRUE(found);
    }
}

TEST(FindOpenCirclesTest, test_find_open_circle_three_in_rectangle)
{
    Rectangle rectangle = Field::createSSLDivisionBField().fieldLines();

    std::vector<Point> points         = {Point(-1, -1), Point(1, -1), Point(0, 1)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(8, empty_circles.size());

    // Calculated from Voronoi diagram
    EXPECT_EQ(-4.5, empty_circles[2].origin().x());
    EXPECT_NEAR(2.09, empty_circles[2].origin().y(), 0.05);
    EXPECT_NEAR(4.628, empty_circles[2].radius(), 0.005);

    EXPECT_EQ(4.5, empty_circles[3].origin().x());
    EXPECT_NEAR(2.09, empty_circles[3].origin().y(), 0.05);
    EXPECT_NEAR(4.628, empty_circles[3].radius(), 0.005);

    EXPECT_EQ(Point(0, -3), empty_circles[6].origin());
    EXPECT_NEAR(2.236, empty_circles[6].radius(), 0.005);

    EXPECT_EQ(Point(0, -0.25), empty_circles[7].origin());
    EXPECT_NEAR(1.25, empty_circles[7].radius(), 0.005);

    // Corner Points
    EXPECT_EQ(Point(-4.5, 3), empty_circles[0].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(4.5, 2) + std::pow(2, 2)),
                     empty_circles[0].radius());

    EXPECT_EQ(Point(4.5, 3), empty_circles[1].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(4.5, 2) + std::pow(2, 2)),
                     empty_circles[1].radius());

    EXPECT_EQ(Point(-4.5, -3), empty_circles[4].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(3.5, 2) + std::pow(2, 2)),
                     empty_circles[4].radius());

    EXPECT_EQ(Point(4.5, -3), empty_circles[5].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(3.5, 2) + std::pow(2, 2)),
                     empty_circles[5].radius());
}

TEST(FindOpenCirclesTest, test_find_open_circle_points_outside_of_box_one_in_box)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Point> points         = {Point(-2, -1), Point(3, -2), Point(-1.1, -1.1),
                                 Point(0.9, 0.9)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(4, empty_circles.size());

    EXPECT_EQ(Point(-1, -1), empty_circles[0].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(1.9, 2)),
                     empty_circles[0].radius());

    EXPECT_EQ(Point(-1, 1), empty_circles[1].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[1].radius());

    EXPECT_EQ(Point(1, 1), empty_circles[2].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(0.1, 2) + std::pow(0.1, 2)),
                     empty_circles[2].radius());

    EXPECT_EQ(Point(1, -1), empty_circles[3].origin());
    EXPECT_DOUBLE_EQ(std::sqrt(std::pow(1.9, 2) + std::pow(0.1, 2)),
                     empty_circles[3].radius());
}

TEST(FindOpenCirclesTest, test_find_open_circle_points_outside_of_box)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));

    std::vector<Point> points         = {Point(-2, -1), Point(3, -2), Point(-1.1, -1.1)};
    std::vector<Circle> empty_circles = findOpenCircles(rectangle, points);

    ASSERT_EQ(0, empty_circles.size());
}

TEST(FindClosestPointTest, test_find_closest_point_zero_points)
{
    std::vector<Point> test_points = {};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(std::nullopt, findClosestPoint(reference_point, test_points));
}

TEST(FindClosestPointTest, test_find_closest_point_one_point)
{
    std::vector<Point> test_points = {Point(-2, -1)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[0], findClosestPoint(reference_point, test_points));
}

TEST(FindClosestPointTest, test_find_closest_point_two_points)
{
    std::vector<Point> test_points = {Point(-2, -1), Point(0.7, 0.6)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[1], findClosestPoint(reference_point, test_points));
}

TEST(FindClosestPointTest, test_find_closest_point_two_points_the_same)
{
    std::vector<Point> test_points = {Point(0.7, 0.6), Point(0.7, 0.6)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[0], findClosestPoint(reference_point, test_points));
}

TEST(FindClosestPointTest, test_find_closest_point_many_points)
{
    std::vector<Point> test_points = {Point(0.7, 0.6), Point(0.8, 0.6), Point(-0.7, -0.6),
                                      Point(0.1, 0.2), Point(-1, -3.4)};
    Point reference_point(0.9, 0.9);
    EXPECT_EQ(test_points[1], findClosestPoint(reference_point, test_points));
}
