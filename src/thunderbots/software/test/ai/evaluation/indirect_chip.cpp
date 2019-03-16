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

TEST(IndirectChipAndChaseTargetTest, triangle_not_empty_and_target_within_reach_test)
{
    std::vector<Triangle> triangles;
    Triangle t = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(0, (-1 + sqrt(0.75) - 1) / 3);
    target       = target.norm((target - ball_position).len() * 0.85);

    EXPECT_EQ(target, Evaluation::findTargetPointForIndirectChipAndChase(triangles,
                                                                         ball_position));
}


TEST(IndirectChipAndChaseTargetTest, triangle_not_empty_and_target_not_within_reach_test)
{
    std::vector<Triangle> triangles;
    Triangle t = {Point(8, 5.9), Point(8.5, 6), Point(8.9, 5.9)};
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(25.4 / 3, 17.8 / 3);
    target       = target.norm((target - ball_position).len() * 0.85);
    target       = ball_position + (target - ball_position).norm(8.0);

    EXPECT_EQ(std::optional(target), Evaluation::findTargetPointForIndirectChipAndChase(
                                         triangles, ball_position));
}


TEST(IndirectChipAndChaseTargetTest, triangle_is_empty_test)
{
    std::vector<Triangle> triangles;

    Point ball_position = Point(0, 0);

    EXPECT_EQ(std::nullopt, Evaluation::findTargetPointForIndirectChipAndChase(
                                triangles, ball_position));
}


TEST(GetAllTrianglesTest, get_all_triangles_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    std::vector<Point> enemy_players;
    enemy_players.emplace_back(Point(1, 2));

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

    EXPECT_EQ(all_triangles, Evaluation::getAllTrianglesBetweenEnemyPlayers(test_world, enemy_players));
}


TEST(FilterOpenTrianglesTest, filter_open_triangles_test)
{
    std::vector<Triangle> triangles;

    Triangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<Point> enemy_players;

    Point enemy_player1 = Point(-0.7, -0.7);
    enemy_players.emplace_back(enemy_player1);

    std::vector<Triangle> filtered_triangles = triangles;
    filtered_triangles.pop_back();

    EXPECT_EQ(filtered_triangles,
              Evaluation::findOpenTriangles(triangles, enemy_players));
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


TEST(GetTriangleCenterAndAreaTest, get_triangle_center_test)
{
    Triangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1          = triangle[0];
    Point p2          = triangle[1];
    Point p3          = triangle[2];

    Point center = Point(0, (-1 + sqrt(0.75) - 1) / 3);

    EXPECT_EQ(center, Evaluation::getTriangleCenter(triangle));
}


TEST(GetTriangleCenterAndAreaTest, get_triangle_area_test)
{
    Triangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1          = triangle[0];
    Point p2          = triangle[1];
    Point p3          = triangle[2];

    double area  = abs(0.5 * (2 * (1 + sqrt(0.75))));

    EXPECT_EQ(area, Evaluation::getTriangleArea(triangle));
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

    corners.emplace_back(Point(ballX, negFieldY));
    corners.emplace_back(Point(ballX, posFieldY));
    corners.emplace_back(Point(fieldX, negFieldY));
    corners.emplace_back(Point(fieldX, posFieldY));

    EXPECT_EQ(corners, Evaluation::findBestChipTargetArea(test_world, inset));
}


TEST(GetLargestTriangleTest, get_largest_triangle_test)
{
    std::vector<Triangle> allTriangles;

    Triangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    allTriangles.emplace_back(t1);
    allTriangles.emplace_back(t2);

    Triangle largest = t2;

    EXPECT_EQ(std::optional(largest), Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0));
}


TEST(GetLargestTriangleTest, get_largest_triangle_with_empty_vector_of_triangles_test)
{
    std::vector<Triangle> allTriangles;

    EXPECT_EQ(std::nullopt, Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0));
}
