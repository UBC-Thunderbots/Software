#include "software/geom/algorithms/rasterize.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/test_util/test_util.h"

namespace TestUtil
{
    void checkPointsCloseToEachOther(std::vector<Point> all_points, double max_dist)
    {
        for (Point& p : all_points)
        {
            auto compare_based_on_distance_to_p = [p](const Point& a,
                                                      const Point& b) -> bool {
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

    void checkPolygonPointsInsideBoundingBox(Polygon polygon,
                                             std::vector<Point> all_points)
    {
        const std::vector<Point>& polygon_vertices = polygon.getPoints();
        auto max_point_y = [](const Point& a, const Point& b) { return a.y() < b.y(); };

        auto max_point_x = [](const Point& a, const Point& b) { return a.x() < b.x(); };

        // Calculate the highest and lowest x and y points
        double min_y = std::min_element(polygon_vertices.begin(), polygon_vertices.end(),
                                        max_point_y)
                           ->y();
        double min_x = std::min_element(polygon_vertices.begin(), polygon_vertices.end(),
                                        max_point_x)
                           ->x();
        double max_y = std::max_element(polygon_vertices.begin(), polygon_vertices.end(),
                                        max_point_y)
                           ->y();
        double max_x = std::max_element(polygon_vertices.begin(), polygon_vertices.end(),
                                        max_point_x)
                           ->x();

        Rectangle bounding_box = Rectangle(Point(min_x, min_y), Point(max_x, max_y));
        for (Point p : all_points)
        {
            EXPECT_TRUE(contains(bounding_box, p));
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
    Circle circle({0, 0}, 3.f);
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

TEST(RasterizeTest, test_circle_not_at_origin)
{
    Circle circle({-2.f, -2.f}, 1.5f);
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

TEST(RasterizeTest, test_circle_with_complex_dimensions)
{
    // Long random floating point numbers are chosen to make sure that points do not
    // exceed the boundary of the circle due to floating point error
    Circle circle({4.6318964f, 7.4893115f}, 2.5498456f);
    double pixel_size                    = 0.1562131f;
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

TEST(RasterizeTest, test_rectangle_not_at_origin)
{
    Rectangle rectangle(Point(-1, -1), Point(1, 1));
    double pixel_size                    = 0.33f;
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

TEST(RasterizeTest, test_rectangle_with_complex_floating_dimensions)
{
    // Long random floating point numbers are chosen to make sure that points do not
    // exceed the boundary of the rectangle due to floating point error
    Rectangle rectangle(Point(-1.3245648f, 1.4895349f), Point(0.4563189f, 1.1234567f));
    double pixel_size                    = 0.5489654f;
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
TEST(RasterizeTest, test_rasterize_polygon)
{
    Polygon polygon{Point(0, 5), Point(0, 0), Point(5, 0), Point(5, 5)};
    double offset                        = 1;
    std::vector<Point> rasterized_points = rasterize(polygon, offset);

    TestUtil::checkPolygonPointsInsideBoundingBox(polygon, rasterized_points);
    TestUtil::checkPointsCloseToEachOther(rasterized_points, offset);
}

TEST(RasterizeTest, test_rasterize_polygon_complex)
{
    Polygon polygon{Point(3.018497, -1.481503), Point(3.018497, 1.481503),
                    Point(4.7999999999999998, 1.481503),
                    Point(4.7999999999999998, -1.481503)};
    double offset = 0.09f;

    std::vector<Point> rasterized_points = rasterize(polygon, offset);

    TestUtil::checkPolygonPointsInsideBoundingBox(polygon, rasterized_points);
    TestUtil::checkPointsCloseToEachOther(rasterized_points, offset);
}

TEST(RasterizeTest, test_rasterize_hexagon)
{
    Polygon hexagon{{0.0f, 2.0f},    // top vertex
                    {1.8f, 1.0f},    // top right vertex
                    {1.8f, -1.0f},   // bottom right vertex
                    {0.0f, -2.0f},   // bottom vertex
                    {-2.0f, -1.0f},  // bottom left vertex
                    {-2.0f, 1.0f}};  // top left vertex
    double offset = 0.5f;

    std::vector<Point> rasterized_points = rasterize(hexagon, offset);

    TestUtil::checkPolygonPointsInsideBoundingBox(hexagon, rasterized_points);
    TestUtil::checkPointsCloseToEachOther(rasterized_points, offset);
}

TEST(RasterizeTest, test_rasterize_complex_self_intersecting_polygon)
{
    Polygon intersecting_poly{{-3.0f, 0.0f}, {-3.0f, 3.0f}, {3.0f, 3.0f},
                              {3.0f, 0.0f},  {-2.0f, 2.0f}, {2.0f, 2.0f}};
    double offset = 0.5f;

    std::vector<Point> rasterized_points = rasterize(intersecting_poly, offset);

    TestUtil::checkPolygonPointsInsideBoundingBox(intersecting_poly, rasterized_points);
    TestUtil::checkPointsCloseToEachOther(rasterized_points, offset);
}
