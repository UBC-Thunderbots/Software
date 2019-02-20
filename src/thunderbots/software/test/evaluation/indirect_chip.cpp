/**
 * Unit tests for indirect_chip_and_chase_target evaluation function and its related functions.
 */
#include <gtest/gtest.h>

#include "ai/hl/stp/evaluation/indirect_chip.h"
#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/util.h"
#include "test/test_util/test_util.h"

const double CHIP_CHERRY_POWER_DOWNSCALE(0.85);
const double MAX_CHIP_POWER(8.0);

/*TEST(IndirectChipAndChaseTargetTest, target_within_reach)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    Robot enemy_goalie = test_world.enemyTeam().goalie().value();
    std::vector<Point> non_goalie_enemy_positions;
    std::vector<Point> all_enemy_positions;

    for (Robot i : test_world.enemyTeam().getAllRobots())
    {
        all_enemy_positions.push_back(i.position());
        if (i != enemy_goalie)
        {
            non_goalie_enemy_positions.push_back(i.position());
        }
    }

    std::vector<Triangle> allTriangles =
        Evaluation::get_all_triangles(test_world, non_goalie_enemy_positions);

    std::vector<Triangle> target_triangles =
        Evaluation::filter_open_triangles(allTriangles, all_enemy_positions);

    target_triangles = Evaluation::remove_outofbounds_triangles(test_world, target_triangles);

    if (!target_triangles.empty())
    {
        std::pair<Triangle, bool> largest_triangle = Evaluationget_largest_triangle(
            target_triangles, MIN_CHIP_TRI_AREA, MIN_CHIP_TRI_EDGE_LEN);
        Triangle t   = largest_triangle.first;
        bool valid   = largest_triangle.second;
        Point target = get_triangle_center_and_area(t).first;
        target       = target.norm((target - world.ball().position()).len() *
                             CHIP_CHERRY_POWER_DOWNSCALE);


    // A non-empty target triangle
    std::vector<Triangle> test_target_triangle(1);

    TestUtil::setBallPosition(test_world, Point(0, 0), Timestamp::fromMilliseconds(0));

    Point target = Point(0, 0);
    target       = target.norm((target - test_world.ball().position()).len() *
                         CHIP_CHERRY_POWER_DOWNSCALE);

    // Expected output: target point, true
    std::pair<Point, bool> test_pair = std::make_pair(target, true);
    EXPECT_EQ(test_pair, Evaluation::indirect_chip_and_chase_target(test_world));

    ////////////
    Robot enemy_goalie = world.enemyTeam().goalie().value();
    std::vector<Point> non_goalie_enemy_positions;
    std::vector<Point> all_enemy_positions;

    for (Robot i : world.enemyTeam().getAllRobots())
    {
        all_enemy_positions.push_back(i.position());
        if (i != enemy_goalie)
        {
            non_goalie_enemy_positions.push_back(i.position());
        }
    }

    std::vector<Triangle> allTriangles =
        get_all_triangles(world, non_goalie_enemy_positions);

    std::vector<Triangle> target_triangles =
        filter_open_triangles(allTriangles, all_enemy_positions);

    target_triangles = remove_outofbounds_triangles(world, target_triangles);

    if (!target_triangles.empty())
    {
        std::pair<Triangle, bool> largest_triangle = get_largest_triangle(
            target_triangles, MIN_CHIP_TRI_AREA, MIN_CHIP_TRI_EDGE_LEN);
        Triangle t   = largest_triangle.first;
        bool valid   = largest_triangle.second;
        Point target = get_triangle_center_and_area(t).first;
        target       = target.norm((target - world.ball().position()).len() *
                             CHIP_CHERRY_POWER_DOWNSCALE);

}

TEST(IndirectChipAndChaseTargetTest, target_further_than_max_chip_power)
{
    // Given a target triangle
    std::vector<Triangle> test_target_triangle(1);

    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    Point target = Point(3, 3);
    target       = target.norm((target - test_world.ball().position()).len() *
                         CHIP_CHERRY_POWER_DOWNSCALE);
                         
    // If target is further away than MAX_CHIP_POWER, adjust target
    if ((target - test_world.ball().position()).len() > MAX_CHIP_POWER)
        target = test_world.ball().position() +
                 (target - test_world.ball().position()).norm(MAX_CHIP_POWER);

    // Expected output: target point, true (valid)
    std::pair<Point, bool> test_pair = std::make_pair(target, true);
    EXPECT_EQ(test_pair, Evaluation::indirect_chip_and_chase_target(test_world));
}

TEST(IndirectChipAndChaseTargetTest, target_triangle_is_empty)
{
    // Given an empty target triangle
    std::vector<Triangle> test_target_triangle;

    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();

    // Expected output: position of enemy goal, false
    std::pair<Point, bool> test_pair = std::make_pair(test_world.field().enemyGoal(), false);
    EXPECT_EQ(test_pair, Evaluation::indirect_chip_and_chase_target(test_world));
}

TEST(GetAllTrianglesTest, get_all_triangles_test)
{
    using namespace Test;
    World test_world = TestUtil::createBlankTestingWorld();


}

TEST(FilterOpenTrianglesTest, filter_open_triangles_test)
{

}
*/

TEST(GetTriangleCenterAndAreaTest, get_triangle_center_and_area_test)
{
    Triangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1 = triangle[0];
    Point p2 = triangle[1];
    Point p3 = triangle[2];

    Point center = Point(0, (-1 + sqrt(0.75) - 1)/3);
    double area  = abs(0.5 * (2 * 1));

    std::pair<Point, double> test_pair = std::make_pair(center, area);
    EXPECT_EQ(test_pair, Evaluation::get_triangle_center_and_area(triangle));
}

/*TEST(RemoveOutOfBoundsTrianglesTest, remove_out_of_bounds_triangles_test)
{

}

TEST(GetChipTargetAreaCornersTest, get_chip_target_area_corners_test)
{

}

TEST(GetLargestTriangleTest, get_largest_triangle_test)
{

}
*/

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
