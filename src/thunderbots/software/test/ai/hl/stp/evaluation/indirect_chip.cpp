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

TEST(findTargetPointForIndirectChipAndChaseTest,
     triangle_not_empty_and_target_within_reach_test)
{
    std::vector<LegacyTriangle> triangles;
    LegacyTriangle t = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(0, (-1 + sqrt(0.75) - 1) / 3);
    target       = target.norm((target - ball_position).len() * 0.85);

    EXPECT_EQ(target, Evaluation::findTargetPointForIndirectChipAndChase(triangles,
                                                                         ball_position));
}


TEST(findTargetPointForIndirectChipAndChaseTest,
     triangle_not_empty_and_target_not_within_reach_test)
{
    std::vector<LegacyTriangle> triangles;
    LegacyTriangle t = {Point(8, 5.9), Point(8.5, 6), Point(8.9, 5.9)};
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(25.4 / 3, 17.8 / 3);
    target       = target.norm((target - ball_position).len() * 0.85);
    target       = ball_position + (target - ball_position).norm(8.0);

    EXPECT_EQ(std::optional(target), Evaluation::findTargetPointForIndirectChipAndChase(
                                         triangles, ball_position));
}


TEST(findTargetPointForIndirectChipAndChaseTest, triangle_is_empty_test)
{
    std::vector<LegacyTriangle> triangles;

    Point ball_position = Point(0, 0);

    EXPECT_EQ(std::nullopt, Evaluation::findTargetPointForIndirectChipAndChase(
                                triangles, ball_position));
}


TEST(getAllTrianglesBetweenEnemyPlayersTest, get_all_triangles_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    std::vector<Point> enemy_players;
    enemy_players.emplace_back(Point(1, 2));

    std::vector<LegacyTriangle> all_triangles = {
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

    EXPECT_EQ(all_triangles,
              Evaluation::getAllTrianglesBetweenEnemyPlayers(test_world, enemy_players));
}


TEST(findOpenTrianglesTest, find_open_triangles_test)
{
    std::vector<LegacyTriangle> triangles;
    LegacyTriangle t1 = {Point(-1.8, -1.8), Point(0, 0.5), Point(0.5, -0.5)};
    LegacyTriangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<Point> enemy_players;
    Point enemy_player1 = Point(-1.5, -1.5);
    enemy_players.emplace_back(enemy_player1);

    LegacyTriangle adjusted_triangle = {
        (Point(-1, -1) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(-1, -1)).norm(2.5 * 0.09)),
        (Point(0, sqrt(0.75)) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(0, sqrt(0.75))).norm(2.5 * 0.09)),
        (Point(1, -1) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(1, -1)).norm(2.5 * 0.09))};

    std::vector<LegacyTriangle> open_triangles;
    open_triangles.emplace_back(adjusted_triangle);

    EXPECT_EQ(open_triangles, Evaluation::findOpenTriangles(triangles, enemy_players));
}


TEST(removeTrianglesOutsideRectangleTest, remove_triangles_outside_rectangle_test)
{
    Rectangle target_rectangle = Rectangle(Point(-2, -2), Point(2, 2));

    LegacyTriangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    LegacyTriangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};

    std::vector<LegacyTriangle> triangles;
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<LegacyTriangle> valid_triangles;
    valid_triangles.emplace_back(t1);
    valid_triangles.emplace_back(t2);

    EXPECT_EQ(valid_triangles,
              Evaluation::removeTrianglesOutsideRectangle(target_rectangle, triangles));
}


TEST(GetTriangleCenterTest, get_triangle_center_test)
{
    LegacyTriangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1                = triangle[0];
    Point p2                = triangle[1];
    Point p3                = triangle[2];

    Point center = Point(0, (-1 + sqrt(0.75) - 1) / 3);

    EXPECT_EQ(center, Evaluation::getTriangleCenter(triangle));
}


TEST(GetTriangleAreaTest, get_triangle_area_test)
{
    LegacyTriangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1                = triangle[0];
    Point p2                = triangle[1];
    Point p3                = triangle[2];

    double area = abs(0.5 * (2 * (1 + sqrt(0.75))));

    EXPECT_EQ(area, Evaluation::getTriangleArea(triangle));
}


TEST(findBestChipTargetAreaTest, find_best_chip_target_area_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();
    double inset     = 0.3;

    double ballX  = 0;
    double fieldX = 6 - 1.5 - inset;
    // distance from centre to end of field - goal width - inset
    double negFieldY = -3 + inset;
    double posFieldY = 3 - inset;

    Rectangle target_rectangle =
        Rectangle(Point(ballX, negFieldY), Point(fieldX, posFieldY));

    EXPECT_EQ(target_rectangle, Evaluation::findBestChipTargetArea(test_world, inset));
}


TEST(getLargestValidTriangleTest, get_largest_valid_triangle_test)
{
    std::vector<LegacyTriangle> allTriangles;

    LegacyTriangle t1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    LegacyTriangle t2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    allTriangles.emplace_back(t1);
    allTriangles.emplace_back(t2);

    LegacyTriangle largest = t2;

    EXPECT_EQ(std::optional(largest),
              Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0));
}


TEST(getLargestValidTriangleTest,
     get_largest_triangle_with_empty_vector_of_triangles_test)
{
    std::vector<LegacyTriangle> allTriangles;

    EXPECT_EQ(std::nullopt, Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0));
}
