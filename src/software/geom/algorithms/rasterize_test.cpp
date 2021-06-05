#include "software/geom/algorithms/rasterize.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

#include "software/test_util/test_util.h"
#include "software/logger/logger.h"

namespace TestUtil
{
    void checkPointsCloseToEachOther(std::vector<Point> all_points, double max_dist)
    {
        for (Point &p : all_points)
        {
            auto compare_based_on_distance_to_p = [p](const Point &a,
                                                      const Point &b) -> bool {
                return distance(a, p) < distance(b, p);
            };

            std::vector<Point> all_points_copy = all_points;
            all_points_copy.erase(
                std::remove(all_points_copy.begin(), all_points_copy.end(), p),
                all_points_copy.end());
            Point closest_p =
                *std::min_element(all_points_copy.begin(), all_points_copy.end(),
                                  compare_based_on_distance_to_p);
            EXPECT_LE(distance(p, closest_p), max_dist);
        }
    }
};  // namespace TestUtil

//////////////////////////////////////////////////////
////              Testing Circles                 ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_circle_with_pixel_size_not_a_multiple)
{
    Circle circle({0, 0}, 1.5f);
    double pixel_size                    = 1.f;
    std::vector<Point> rasterized_points = rasterize(circle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(circle, p));
    }
    // sqrt(2) * pixel_size since the circle rasterize algorithm does not guarantee that
    // the points are all aligned in x and y axis.
    double max_dist = std::sqrt(2) * pixel_size;
    TestUtil::checkPointsCloseToEachOther(rasterized_points, max_dist);
}

TEST(RasterizeTest, test_circle_with_actual_theta_star_values)
{
    double robot_obstacle = 2 * 0.085f;
    Circle circle({0, 0}, robot_obstacle);
    double theta_star_grid_square_size   = 0.09;
    std::vector<Point> rasterized_points = rasterize(circle, theta_star_grid_square_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(circle, p));
    }
    // sqrt(2) * pixel_size since the circle rasterize algorithm does not guarantee that
    // the points are all aligned in x and y axis.
    double max_dist = std::sqrt(2) * theta_star_grid_square_size;
    TestUtil::checkPointsCloseToEachOther(rasterized_points, max_dist);
}

TEST(RasterizeTest, test_large_circle)
{
    double robot_obstacle = 3.f;
    Circle circle({0, 0}, robot_obstacle);
    double pixel_size                    = 0.1f;
    std::vector<Point> rasterized_points = rasterize(circle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(circle, p)) << p;
    }
    // sqrt(2) * pixel_size since the circle rasterize algorithm does not guarantee that
    // the points are all aligned in x and y axis.
    double max_dist = std::sqrt(2) * pixel_size;
    TestUtil::checkPointsCloseToEachOther(rasterized_points, max_dist);
}

//////////////////////////////////////////////////////
////            Testing Rectangles                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_pixel_size_multiple_of_square_dimensions)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double pixel_size                    = 0.5f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    TestUtil::checkPointsCloseToEachOther(rasterized_points, pixel_size);
}

TEST(RasterizeTest, test_pixel_size_larger_than_square)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double pixel_size                    = 2;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    TestUtil::checkPointsCloseToEachOther(rasterized_points, pixel_size);
}

TEST(RasterizeTest, test_pixel_size_not_a_multiple_of_square_dimensions)
{
    // Rectangle dimensions are a bit larger than pixel size so an extra point should be
    // added in each dimension
    Rectangle rectangle(Point(0, 0), Point(2.01f, 2.01f));
    double pixel_size                    = 0.5f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    TestUtil::checkPointsCloseToEachOther(rasterized_points, pixel_size);
}

TEST(RasterizeTest, test_pixel_size_one_dimesnsion_not_multiple_of_rectangle_dimensions)
{
    // Only one dimension is smaller than pixel size. The two dimensions are different
    Rectangle rectangle(Point(0, 0), Point(0.9f, 2.0f));
    double pixel_size                    = 1.0f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    TestUtil::checkPointsCloseToEachOther(rasterized_points, pixel_size);
}

//////////////////////////////////////////////////////
////              Testing Polygons                ////
//////////////////////////////////////////////////////
// TODO could check that the points are offsetted by pixel_size!?
TEST(RasterizeTest, test_rasterize_polygon)
{
	std::vector<Point> points = { Point(0, 5), Point(0, 0), Point(5, 0), Point(5, 5) };
    double offset = 1;
    Polygon polygon = Polygon(points);
    std::vector<Point> rasterized_points = rasterize(polygon, offset);

    for (Point p : rasterized_points)
    {
        bool result = contains(polygon, p);
        //TODO: remove this if statement
        if (!result)
        {
            std::cout << "FAILED: " << p << "\n";
        }
        EXPECT_TRUE(contains(polygon, p));
    }
}

TEST(RasterizeTest, test_rasterize_polygon_complex)
{
    std::vector<Point> points = {Point(3.018497, -1.481503), Point(3.018497, 1.481503),
        Point(4.7999999999999998, 1.481503), Point(4.7999999999999998, -1.481503)};
    Polygon polygon = Polygon(points);

    std::vector<Point> rasterized_points = rasterize(polygon, 0.09);
    for (Point p : rasterized_points)
    {
        bool result = contains(polygon, p);
        //TODO: Remove std::cout
        if (!result)
        {
            std::cout << "FAILED: " << p << "\n";
        }
        EXPECT_TRUE(contains(polygon, p));
    }
}

TEST(RasterizeTest, test_speed_polygon)
{
    std::vector<Point> points = { Point(0, 5), Point(0, 0), Point(5, 0), Point(5, 5) };
    double offset = 0.1f;
    Polygon polygon = Polygon(points);
    auto start_tick_time = std::chrono::system_clock::now();
    std::vector<Point> rasterized_points = rasterize(polygon, offset);
    double duration_ms         = ::TestUtil::millisecondsSince(start_tick_time);
    LOG(WARNING) << "max tick duration: " << duration_ms << "ms" << std::endl;

    for (Point p : rasterized_points)
    {
        bool result = contains(polygon, p);
        //TODO: remove this if statement
        if (!result)
        {
            std::cout << "FAILED: " << p << "\n";
        }
        EXPECT_TRUE(contains(polygon, p));
    }
}

TEST(RasterizeTest, test_speed_rectangle)
{
    double offset = 0.1f;
    Rectangle rectangle(Point(0,0), Point(5,5));
    auto start_tick_time = std::chrono::system_clock::now();
    std::vector<Point> rasterized_points = rasterize(rectangle, offset);
    double duration_ms         = ::TestUtil::millisecondsSince(start_tick_time);
    LOG(WARNING) << "max tick duration: " << duration_ms << "ms" << std::endl;

    for (Point p : rasterized_points)
    {
        bool result = contains(rectangle, p);
        //TODO: remove this if statement
        if (!result)
        {
            std::cout << "FAILED: " << p << "\n";
        }
        EXPECT_TRUE(contains(rectangle, p));
    }
}