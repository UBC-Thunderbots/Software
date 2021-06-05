#include "software/geom/algorithms/rasterize.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"

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
    EXPECT_EQ(rasterized_points.size(), 9);
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
    EXPECT_EQ(rasterized_points.size(), 9);
}

TEST(RasterizeTest, test_large_circle)
{
    double robot_obstacle = 3.f;
    Circle circle({0, 0}, robot_obstacle);
    double theta_star_grid_square_size   = 0.1f;
    std::vector<Point> rasterized_points = rasterize(circle, theta_star_grid_square_size);

    for (Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(circle, p)) << p;
    }
    EXPECT_EQ(rasterized_points.size(), 9);
}


//////////////////////////////////////////////////////
////              Testing Polygons                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_polygon_triangle_contains_point)
{
    // Hexagon centered at origin with the following points
    Polygon hexagon{{0.0f, 2.0f},    // top vertex
                    {2.0f, 1.0f},    // top right vertex
                    {2.0f, -1.0f},   // bottom right vertex
                    {0.0f, -2.0f},   // bottom vertex
                    {-2.0f, -1.0f},  // bottom left vertex
                    {-2.0f, 1.0f}};  // top left vertex

    double pixel_size                    = 1.f;
    std::vector<Point> rasterized_points = rasterize(hexagon, pixel_size);
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
    EXPECT_EQ(rasterized_points.size(), 9);
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
    // Should always have atleast the 4 corners of the rectangle
    EXPECT_EQ(rasterized_points.size(), 4);
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
    EXPECT_EQ(rasterized_points.size(), 36);
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
    EXPECT_EQ(rasterized_points.size(), 6);
}
// TODO could check that the points are offsetted by pixel_size!?
