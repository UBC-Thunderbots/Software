#include "software/geom/algorithms/rasterize.h"
#include "software/geom/algorithms/contains.h"

#include <gtest/gtest.h>


//////////////////////////////////////////////////////
////              Testing Circles                 ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, segment_in_circle)
{
//    Circle c({1.0, 1.0}, 4.0);
}

//////////////////////////////////////////////////////
////              Testing Polygons                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_polygon_triangle_contains_point)
{
//    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
}

//////////////////////////////////////////////////////
////            Testing Rectangles                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_pixel_size_multiple_of_square_dimensions)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double pixel_size = 0.5f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for(Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    EXPECT_EQ(rasterized_points.size(), 9);
}

TEST(RasterizeTest, test_pixel_size_larger_than_square)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double pixel_size = 2;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for(Point p : rasterized_points)
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
    double pixel_size = 0.5f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for(Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    EXPECT_EQ(rasterized_points.size(), 36);
}

TEST(RasterizeTest, test_pixel_size_one_dimesnsion_not_multiple_of_rectangle_dimensions)
{
    // Only one dimension is smaller than pixel size. The two dimensions are different
    Rectangle rectangle(Point(0, 0), Point(0.9f, 2.0f));
    double pixel_size = 1.0f;
    std::vector<Point> rasterized_points = rasterize(rectangle, pixel_size);

    for(Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    EXPECT_EQ(rasterized_points.size(), 6);
}
// TODO could check that the points are offsetted by pixel_size!?