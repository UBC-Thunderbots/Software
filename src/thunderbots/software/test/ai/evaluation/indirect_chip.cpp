/**
 * Unit tests for indirect_chip_and_chase_target evaluation function and its related
 * functions.
 */
#include "ai/hl/stp/evaluation/indirect_chip.h"

#include <gtest/gtest.h>

#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/util.h"
#include "test/test_util/test_util.h"

TEST(GetAllTrianglesTest, get_all_triangles_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    std::vector<Point> enemy_players;
    enemy_players.push_back(Point(1, 2));

    std::vector<Triangle> all_triangles = {
        {Point(1, 2), Point(6 - 1.5, -3), Point(6 - 1.5, 3)},
        {Point(1, 2), Point(6 - 1.5, -3), Point(0, 3)},
        {Point(1, 2), Point(6 - 1.5, -3), Point(0, -3)},
        {Point(1, 2), Point(6 - 1.5, 3), Point(0, 3)},
        {Point(1, 2), Point(6 - 1.5, 3), Point(0, -3)},
        {Point(1, 2), Point(0, 3), Point(0, -3)},
        {Point(6 - 1.5, -3), Point(6 - 1.5, 3), Point(0, 3)},
        {Point(6 - 1.5, -3), Point(6 - 1.5, 3), Point(0, -3)},
        {Point(6 - 1.5, -3), Point(0, 3), Point(0, -3)},
        {Point(6 - 1.5, 3), Point(0, 3), Point(0, -3)}};

    EXPECT_EQ(all_triangles, Evaluation::get_all_triangles(test_world, enemy_players));
}


TEST(FilterOpenTrianglesTest, filter_open_triangles_test)
{
    std::vector<Triangle> triangles;

    Triangle triangle1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle triangle2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(triangle1);
    triangles.emplace_back(triangle2);

    std::vector<Point> enemy_players;

    Point enemy_player1 = Point(-0.7, -0.7);
    enemy_players.emplace_back(enemy_player1);

    std::vector<Triangle> filtered_triangles = triangles;
    filtered_triangles.pop_back();

    EXPECT_EQ(filtered_triangles,
              Evaluation::filter_open_triangles(triangles, enemy_players));
}


TEST(RemoveOutofboundsTrianglesTest, remove_outofbounds_triangles_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    Triangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};

    std::vector<Triangle> triangles;
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<Triangle> valid_triangles;
    valid_triangles.emplace_back(t1);
    valid_triangles.emplace_back(t2);

    EXPECT_EQ(valid_triangles,
              Evaluation::remove_outofbounds_triangles(test_world, triangles));
}


TEST(GetTriangleCenterAndAreaTest, get_triangle_center_and_area_test)
{
    Triangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1          = triangle[0];
    Point p2          = triangle[1];
    Point p3          = triangle[2];

    Point center = Point(0, (-1 + sqrt(0.75) - 1) / 3);
    double area  = abs(0.5 * (2 * (1 + sqrt(0.75))));

    std::pair<Point, double> test_pair = std::make_pair(center, area);
    EXPECT_EQ(test_pair, Evaluation::get_triangle_center_and_area(triangle));
}


TEST(GetChipTargetAreaCornersTest, get_chip_target_area_corners_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();
    double inset     = 0.3;

    std::vector<Point> corners;

    double ballX  = 0;
    double fieldX = 6 - 1.5 - inset;
    // distance from centre to end of field - goal width - inset
    double negFieldY = -3 + inset;
    double posFieldY = 3 - inset;

    corners.push_back(Point(ballX, negFieldY));
    corners.push_back(Point(ballX, posFieldY));
    corners.push_back(Point(fieldX, negFieldY));
    corners.push_back(Point(fieldX, posFieldY));

    EXPECT_EQ(corners, Evaluation::get_chip_target_area_corners(test_world, inset));
}


TEST(GetLargestTriangleTest, get_largest_triangle_test)
{
    std::vector<Triangle> allTriangles;

    Triangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    allTriangles.emplace_back(t1);
    allTriangles.emplace_back(t2);

    Triangle largest = t2;
    bool valid       = true;

    std::pair<Triangle, bool> test_pair = std::make_pair(largest, valid);
    EXPECT_EQ(test_pair, Evaluation::get_largest_triangle(allTriangles, 0, 0, 0));
}
