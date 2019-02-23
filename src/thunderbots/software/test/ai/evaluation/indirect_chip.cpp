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

TEST(GetLargestTriangleTest, get_largest_triangle_test)
{
    std::vector<Triangle> allTriangles;

    Triangle triangle1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle triangle2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    allTriangles.emplace_back(triangle1);
    allTriangles.emplace_back(triangle2);

    Triangle largest = triangle2;
    bool valid       = true;

    std::pair<Triangle, bool> test_pair = std::make_pair(largest, valid);
    EXPECT_EQ(test_pair, Evaluation::get_largest_triangle(allTriangles, 0, 0, 0));
}

TEST(FilterOpenTrianglesTest, filter_open_triangles_test)
{
    /*Robot friendly_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromMilliseconds(0));
    Robot enemy_robot    = Robot(0, Point(-3, 0), Vector(0, 0), Angle::zero(),
                              AngularVelocity::zero(), Timestamp::fromMilliseconds(0));
    std::vector<Robot> friendly_robots;
    std::vector<Robot> enemy_robots;
    friendly_robots.emplace_back(friendly_robot);
    enemy_robots.emplace_back(enemy_robot);*/

    std::vector<Triangle> triangles;

    Triangle triangle1 = {Point(-0.5, -0.5), Point(0, 0.5), Point(0.5, -0.5)};
    Triangle triangle2 = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    triangles.emplace_back(triangle1);
    triangles.emplace_back(triangle2);

    std::vector<Point> enemy_players;

    Point enemy_player1 = Point(-0.7, -0.7);
    enemy_players.emplace_back(enemy_player1);

    std::vector<Triangle> filtered_triangles = triangles;
    filtered_triangles.push_back(triangle2);

    EXPECT_EQ(filtered_triangles, Evaluation::filter_open_triangles(triangles,enemy_players));
}

/*std::vector<Triangle> Evaluation::filter_open_triangles(std::vector<Triangle> triangles,
                                                        std::vector<Point> enemy_players)
{
    std::vector<Triangle> filtered_triangles;
    bool containsEnemy = false;

    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        // Create a slightly smaller triangle so the robots making up the vertices
        // are not counted in the triangle
        Point p1 = triangles[i][0] +
                   ((get_triangle_center_and_area(triangles[i]).first) - triangles[i][0])
                       .norm(2.5 * ROBOT_MAX_RADIUS_METERS);
        Point p2 = triangles[i][1] +
                   ((get_triangle_center_and_area(triangles[i]).first) - triangles[i][1])
                       .norm(2.5 * ROBOT_MAX_RADIUS_METERS);
        Point p3 = triangles[i][2] +
                   ((get_triangle_center_and_area(triangles[i]).first) - triangles[i][2])
                       .norm(2.5 * ROBOT_MAX_RADIUS_METERS);
        Triangle t    = triangle(p1, p2, p3);
        containsEnemy = false;

        for (unsigned int k = 0; k < enemy_players.size(); k++)
        {
            if (contains(t, enemy_players[k]) == true)
            {
                containsEnemy = true;
                break;
            }
        }

        if (containsEnemy == false)
        {
            filtered_triangles.push_back(triangles[i]);
        }
    }

    return filtered_triangles;
}*/
int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
