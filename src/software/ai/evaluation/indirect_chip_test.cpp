/**
 * Unit tests for indirect_chip_and_chase_target evaluation function and its related
 * functions.
 */
#include "software/ai/evaluation/indirect_chip.h"

#include <gtest/gtest.h>

#include "software/geom/util.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

void testVectorTrianglesEqual(const std::vector<Triangle>& vec1, const std::vector<Triangle>& vec2)
{
    ASSERT_EQ(vec1.size(), vec2.size());
    for (int i = 0; i < vec1.size(); ++i) {
        EXPECT_TRUE(vec1[i].getPoints() == vec2[i].getPoints());
    }
}

TEST(findTargetPointForIndirectChipAndChaseTest,
     triangle_not_empty_and_target_within_reach_test)
{
    std::vector<Triangle> triangles;
    Triangle t = Triangle(Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1));
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(0, (-1 + sqrt(0.75) - 1) / 3);
    target = Point(target.toVector().normalize((target - ball_position).length() * 0.85));

    EXPECT_EQ(target, Evaluation::findTargetPointForIndirectChipAndChase(triangles,
                                                                         ball_position));
}


TEST(findTargetPointForIndirectChipAndChaseTest,
     triangle_not_empty_and_target_not_within_reach_test)
{
    std::vector<Triangle> triangles;
    Triangle t = Triangle(Point(8, 5.9), Point(8.5, 6), Point(8.9, 5.9));
    triangles.emplace_back(t);

    Point ball_position = Point(0, 0);

    Point target = Point(25.4 / 3, 17.8 / 3);
    target = Point(target.toVector().normalize((target - ball_position).length() * 0.85));
    target = ball_position + (target - ball_position).normalize(8.0);

    EXPECT_EQ(std::optional(target), Evaluation::findTargetPointForIndirectChipAndChase(
                                         triangles, ball_position));
}


TEST(findTargetPointForIndirectChipAndChaseTest, triangle_is_empty_test)
{
    std::vector<Triangle> triangles;

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

    std::vector<Triangle> all_triangles = {
        Triangle(Point(1, 2), Point(6 - 1.5, -3), Point(6 - 1.5, 3)),
        Triangle(Point(1, 2), Point(6 - 1.5, -3), Point(0, 3)),
        Triangle(Point(1, 2), Point(6 - 1.5, -3), Point(0, -3)),
        Triangle(Point(1, 2), Point(6 - 1.5, 3), Point(0, 3)),
        Triangle(Point(1, 2), Point(6 - 1.5, 3), Point(0, -3)),
        Triangle(Point(1, 2), Point(0, 3), Point(0, -3)),
        Triangle(Point(6 - 1.5, -3), Point(6 - 1.5, 3), Point(0, 3)),
        Triangle(Point(6 - 1.5, -3), Point(6 - 1.5, 3), Point(0, -3)),
        Triangle(Point(6 - 1.5, -3), Point(0, 3), Point(0, -3)),
        Triangle(Point(6 - 1.5, 3), Point(0, 3), Point(0, -3))};

    std::vector<Triangle> triangles_between_enemies =
            Evaluation::getAllTrianglesBetweenEnemyPlayers(test_world, enemy_players);

    testVectorTrianglesEqual(all_triangles, triangles_between_enemies);
}


TEST(findOpenTrianglesTest, find_open_triangles_test)
{
    std::vector<Triangle> triangles;
    Triangle t1 = Triangle(Point(-1.8, -1.8), Point(0, 0.5), Point(0.5, -0.5));
    Triangle t2 = Triangle(Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1));
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<Point> enemy_players;
    Point enemy_player1 = Point(-1.5, -1.5);
    enemy_players.emplace_back(enemy_player1);

    Triangle adjusted_triangle = Triangle(
        (Point(-1, -1) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(-1, -1)).normalize(2.5 * 0.09)),
        (Point(0, sqrt(0.75)) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(0, sqrt(0.75)))
             .normalize(2.5 * 0.09)),
        (Point(1, -1) +
         (Point(0, (-1 + sqrt(0.75) - 1) / 3) - Point(1, -1)).normalize(2.5 * 0.09)));

    std::vector<Triangle> open_triangles;
    open_triangles.emplace_back(adjusted_triangle);
    std::vector<Triangle> found_open_triangles = Evaluation::findOpenTriangles(triangles, enemy_players);

    testVectorTrianglesEqual(open_triangles, found_open_triangles);
}


TEST(removeTrianglesOutsideRectangleTest, remove_triangles_outside_rectangle_test)
{
    Rectangle target_rectangle = Rectangle(Point(-2, -2), Point(2, 2));

    Triangle t1 = Triangle(Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5));
    Triangle t2 = Triangle(Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1));

    std::vector<Triangle> triangles;
    triangles.emplace_back(t1);
    triangles.emplace_back(t2);

    std::vector<Triangle> valid_triangles;

    valid_triangles.emplace_back(t1);
    valid_triangles.emplace_back(t2);

    std::vector<Triangle> within_chip_triangles =
            Evaluation::removeTrianglesOutsideRectangle(target_rectangle, triangles);

    testVectorTrianglesEqual(valid_triangles, within_chip_triangles);
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
    std::vector<Triangle> allTriangles;

    Triangle t1 = Triangle(Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5));
    Triangle t2 = Triangle(Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1));
    allTriangles.emplace_back(t1);
    allTriangles.emplace_back(t2);

    Triangle largest = t2;

    EXPECT_TRUE(std::optional(largest).value().getPoints() ==
              Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0).value().getPoints());
}


TEST(getLargestValidTriangleTest,
     get_largest_triangle_with_empty_vector_of_triangles_test)
{
    std::vector<Triangle> allTriangles;

    EXPECT_EQ(std::nullopt, Evaluation::getLargestValidTriangle(allTriangles, 0, 0, 0));
}
