#include "indirect_chip.h"

#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/util.h"
#include "shared/constants.h"

std::pair<Point, bool> Evaluation::indirect_chip_and_chase_target(const World& world)
{
    // Creates a vector of all non-goalie enemy robots
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

        // Target should never be further away than MAX_CHIP_POWER
        if ((target - world.ball().position()).len() > MAX_CHIP_POWER)
            target = world.ball().position() +
                     (target - world.ball().position()).norm(MAX_CHIP_POWER);

        return std::make_pair(target, valid);
    }
    else
    {
        return std::make_pair(world.field().enemyGoal(), false);
    }
}

std::vector<Triangle> Evaluation::get_all_triangles(const World& world,
                                                    std::vector<Point> enemy_players)
{
    std::vector<Triangle> triangles;
    std::vector<Point> allPts = enemy_players;

    allPts.push_back(world.field().enemyCornerNeg());
    allPts.push_back(world.field().enemyCornerPos());
    allPts.push_back(Point(0, world.field().enemyCornerPos().y()));
    allPts.push_back(Point(0, world.field().enemyCornerNeg().y()));

    for (unsigned int i = 0; i < allPts.size() - 2; i++)
    {
        for (unsigned int j = i + 1; j < allPts.size() - 1; j++)
        {
            for (unsigned int k = j + 1; k < allPts.size(); k++)
            {
                Point p1   = allPts[i];
                Point p2   = allPts[j];
                Point p3   = allPts[k];
                Triangle t = triangle(p1, p2, p3);
                triangles.push_back(t);
            }
        }
    }

    return triangles;
}

std::vector<Triangle> Evaluation::filter_open_triangles(std::vector<Triangle> triangles,
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
}

std::pair<Point, double> Evaluation::get_triangle_center_and_area(Triangle triangle)
{
    Point p1 = triangle[0];
    Point p2 = triangle[1];
    Point p3 = triangle[2];

    double center_x = (p1.x() + p2.x() + p3.x()) / 3;
    double center_y = (p1.y() + p2.y() + p3.y()) / 3;

    double area  = abs(0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) -
                             (p3.x() - p1.x()) * (p2.y() - p1.y())));
    Point center = Point(center_x, center_y);

    return std::make_pair(center, area);
}

std::vector<Triangle> Evaluation::remove_outofbounds_triangles(
    const World& world, std::vector<Triangle> triangles)
{
    std::vector<Triangle> valid_triangles;
    std::vector<Point> chip_area_corners =
        get_chip_target_area_corners(world, CHIP_TARGET_AREA_INSET);
    Point center;

    double smallest_x = chip_area_corners[0].x();
    double smallest_y = chip_area_corners[0].y();
    double largest_x  = chip_area_corners[0].x();
    double largest_y  = chip_area_corners[0].y();

    for (unsigned int i = 0; i < chip_area_corners.size(); i++)
    {
        Point p = chip_area_corners[i];
        if (p.x() < smallest_x)
            smallest_x = p.x();
        if (p.x() > largest_x)
            largest_x = p.x();
        if (p.y() < smallest_y)
            smallest_y = p.y();
        if (p.y() > largest_y)
            largest_y = p.y();
    }

    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        Triangle t = triangles[i];
        center     = get_triangle_center_and_area(triangles[i]).first;
        if (center.x() <= largest_x && center.x() >= smallest_x && center.y() <= largest_y &&
            center.y() >= smallest_y)
        {
            valid_triangles.push_back(t);
        }
    }

    return valid_triangles;
}

std::vector<Point> Evaluation::get_chip_target_area_corners(const World& world,
                                                            double inset)
{
    std::vector<Point> corners;

    double ballX     = world.ball().position().x();
    double fieldX    = world.field().enemyGoal().x() - inset;
    double negFieldY = world.field().enemyCornerNeg().y() + inset;
    double posFieldY = world.field().enemyCornerPos().y() - inset;

    Point p1 = Point(ballX, negFieldY);
    Point p2 = Point(ballX, posFieldY);
    Point p3 = Point(fieldX, negFieldY);
    Point p4 = Point(fieldX, posFieldY);

    corners.push_back(p1);
    corners.push_back(p2);
    corners.push_back(p3);
    corners.push_back(p4);

    return corners;
}

std::pair<Triangle, bool> Evaluation::get_largest_triangle(
    std::vector<Triangle> allTriangles, double min_area, double min_edge_len,
    double min_edge_angle)
{
    Triangle largest    = allTriangles[0];
    double largest_area = get_triangle_center_and_area(largest).second;
    bool valid          = false;

    for (unsigned int i = 0; i < allTriangles.size(); i++)
    {
        Triangle t  = allTriangles[i];
        double area = get_triangle_center_and_area(t).second;
        double l1   = (t[1] - t[0]).len();
        double l2   = (t[2] - t[0]).len();
        double l3   = (t[2] - t[1]).len();

        Angle a1 = vertexAngle(t[1], t[0], t[2]).angleMod().abs();
        Angle a2 = vertexAngle(t[0], t[1], t[2]).angleMod().abs();
        Angle a3 = vertexAngle(t[0], t[2], t[1]).angleMod().abs();

        if (area > largest_area && area >= min_area && l1 >= min_edge_len &&
            l2 >= min_edge_len && l3 >= min_edge_len &&
            a1.toDegrees() >= min_edge_angle && a2.toDegrees() >= min_edge_angle &&
            a3.toDegrees() >= min_edge_angle)
        {
            largest      = t;
            largest_area = area;
            valid        = true;
        }
    }

    return std::make_pair(largest, valid);
}
