#include "software/ai/evaluation/indirect_chip.h"

#include "shared/constants.h"
#include "software/geom/util.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/world/world.h"

std::optional<Point> Evaluation::findTargetPointForIndirectChipAndChase(
    const World &world)
{
    std::optional<Robot> enemy_goalie_opt = world.enemyTeam().goalie();

    std::vector<Point> non_goalie_enemy_positions;
    std::vector<Point> all_enemy_positions;

    for (Robot i : world.enemyTeam().getAllRobots())
    {
        all_enemy_positions.emplace_back(i.position());
        if (std::optional<Robot>(i) != enemy_goalie_opt)
        {
            non_goalie_enemy_positions.emplace_back(i.position());
        }
    }

    std::vector<Triangle> allTriangles =
        getAllTrianglesBetweenEnemyPlayers(world, non_goalie_enemy_positions);

    std::vector<Triangle> target_triangles =
        findOpenTriangles(allTriangles, all_enemy_positions);

    Rectangle target_area_rectangle =
        findBestChipTargetArea(world, Util::DynamicParameters->getAIConfig()
                                          ->getEvaluationConfig()
                                          ->getIndirectChipConfig()
                                          ->ChipTargetAreaInset()
                                          ->value());

    target_triangles =
        removeTrianglesOutsideRectangle(target_area_rectangle, target_triangles);

    Point ball_position = world.ball().position();

    return findTargetPointForIndirectChipAndChase(target_triangles, ball_position);
}

std::optional<Point> Evaluation::findTargetPointForIndirectChipAndChase(
    const std::vector<Triangle> &triangles, Point ball_position)
{
    if (!triangles.empty())
    {
        // Get the largest triangle within the vector of triangles that has area greater
        // than minimum area of chip target triangle, and all edge lengths greater than
        // minimum edge length of chip target triangle
        std::optional<Triangle> largest_triangle =
            getLargestValidTriangle(triangles,
                                    Util::DynamicParameters->getAIConfig()
                                        ->getEvaluationConfig()
                                        ->getIndirectChipConfig()
                                        ->MinChipTriArea()
                                        ->value(),
                                    Util::DynamicParameters->getAIConfig()
                                        ->getEvaluationConfig()
                                        ->getIndirectChipConfig()
                                        ->MinChipTriEdgeLen()
                                        ->value());
        Triangle t = largest_triangle.value();

        Point target = t.center();
        // Adjust the target point to have a length of distance between itself and the
        // ball's position, then scaling it by a certain percentage
        target = Point(target.toVector().normalize((target - ball_position).length() *
                                                   Util::DynamicParameters->getAIConfig()
                                                       ->getEvaluationConfig()
                                                       ->getIndirectChipConfig()
                                                       ->ChipCherryPowerDownscale()
                                                       ->value()));

        // Target should never be further away than maximum chip power
        if ((target - ball_position).length() > Util::DynamicParameters->getAIConfig()
                                                    ->getEvaluationConfig()
                                                    ->getIndirectChipConfig()
                                                    ->MaxChipPower()
                                                    ->value())
        {
            target = ball_position + (target - ball_position)
                                         .normalize(Util::DynamicParameters->getAIConfig()
                                                        ->getEvaluationConfig()
                                                        ->getIndirectChipConfig()
                                                        ->MaxChipPower()
                                                        ->value());
        }

        return std::optional(target);
    }
    else
    {
        return std::nullopt;
    }
}

std::vector<Triangle> Evaluation::getAllTrianglesBetweenEnemyPlayers(
    const World &world, std::vector<Point> enemy_players)
{
    std::vector<Triangle> all_triangles;
    std::vector<Point> allPts = enemy_players;

    allPts.emplace_back(world.field().enemyCornerNeg());
    allPts.emplace_back(world.field().enemyCornerPos());
    allPts.emplace_back(Point(0, world.field().enemyCornerPos().y()));
    allPts.emplace_back(Point(0, world.field().enemyCornerNeg().y()));

    for (unsigned int i = 0; i < allPts.size() - 2; i++)
    {
        for (unsigned int j = i + 1; j < allPts.size() - 1; j++)
        {
            for (unsigned int k = j + 1; k < allPts.size(); k++)
            {
                // Set up 3 different points from the vector of non-goalie enemy players'
                // positions and the four points for the rectangular region to chip and
                // chase at
                Point p1 = allPts[i];
                Point p2 = allPts[j];
                Point p3 = allPts[k];
                // With the 3 points, create a possible triangle and place in
                // vector of all triangles. Eventually all permutations of points will be
                // picked
                Triangle t = Triangle(p1, p2, p3);
                all_triangles.emplace_back(t);
            }
        }
    }

    return all_triangles;
}

std::vector<Triangle> Evaluation::findOpenTriangles(std::vector<Triangle> triangles,
                                                    std::vector<Point> enemy_players)
{
    std::vector<Triangle> filtered_triangles;

    // For every triangle, the 3 points are adjusted so that the robots making up the
    // vertices won't be counted within the triangle, i.e. make every triangle slightly
    // smaller
    for (Triangle t : triangles)
    {
        const std::vector<Point> &tPoints = t.getPoints();
        // Takes vector of triangles from input and adjust every single triangle within it
        Point p1 = tPoints[0] +
                   (t.center() - tPoints[0]).normalize(2.5 * ROBOT_MAX_RADIUS_METERS);
        Point p2 = tPoints[1] +
                   (t.center() - tPoints[1]).normalize(2.5 * ROBOT_MAX_RADIUS_METERS);
        Point p3 = tPoints[2] +
                   (t.center() - tPoints[2]).normalize(2.5 * ROBOT_MAX_RADIUS_METERS);

        Triangle adjusted_triangle = Triangle(p1, p2, p3);
        bool containsEnemy         = false;

        for (Point enemy_robot : enemy_players)
        {
            if (adjusted_triangle.contains(enemy_robot))
            {
                containsEnemy = true;
                break;
            }
        }

        if (!containsEnemy)
        {
            filtered_triangles.emplace_back(adjusted_triangle);
        }
    }

    return filtered_triangles;
}

std::vector<Triangle> Evaluation::removeTrianglesOutsideRectangle(
    Rectangle rectangle, std::vector<Triangle> triangles)
{
    std::vector<Triangle> valid_triangles;
    Point center;

    double smallest_x = rectangle.negXNegYCorner().x();
    double smallest_y = rectangle.negXNegYCorner().y();
    double largest_x  = rectangle.posXPosYCorner().x();
    double largest_y  = rectangle.posXPosYCorner().y();

    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        Triangle t = triangles[i];
        center     = t.center();
        if (center.x() <= largest_x && center.x() >= smallest_x &&
            center.y() <= largest_y && center.y() >= smallest_y)
        {
            valid_triangles.emplace_back(t);
        }
    }

    return valid_triangles;
}

Rectangle Evaluation::findBestChipTargetArea(const World &world, double inset)
{
    double ballX     = world.ball().position().x();
    double fieldX    = world.field().enemyGoal().x() - inset;
    double negFieldY = world.field().enemyCornerNeg().y() + inset;
    double posFieldY = world.field().enemyCornerPos().y() - inset;

    Rectangle target_rectangle =
        Rectangle(Point(ballX, negFieldY), Point(fieldX, posFieldY));

    return target_rectangle;
}

std::optional<Triangle> Evaluation::getLargestValidTriangle(
    std::vector<Triangle> allTriangles, double min_area, double min_edge_len,
    double min_edge_angle)
{
    if (!(allTriangles.empty()))
    {
        Triangle largest    = allTriangles[0];
        double largest_area = largest.area();

        for (unsigned int i = 0; i < allTriangles.size(); i++)
        {
            Triangle t                        = allTriangles[i];
            double area                       = t.area();
            const std::vector<Point> &tPoints = t.getPoints();
            double l1                         = (tPoints[1] - tPoints[0]).length();
            double l2                         = (tPoints[2] - tPoints[0]).length();
            double l3                         = (tPoints[2] - tPoints[1]).length();

            Angle a1 =
                acuteVertexAngle(tPoints[1], tPoints[0], tPoints[2]).angleMod().abs();
            Angle a2 =
                acuteVertexAngle(tPoints[0], tPoints[1], tPoints[2]).angleMod().abs();
            Angle a3 =
                acuteVertexAngle(tPoints[0], tPoints[2], tPoints[1]).angleMod().abs();

            if (area >= largest_area && area >= min_area && l1 >= min_edge_len &&
                l2 >= min_edge_len && l3 >= min_edge_len &&
                a1.toDegrees() >= min_edge_angle && a2.toDegrees() >= min_edge_angle &&
                a3.toDegrees() >= min_edge_angle)
            {
                largest      = t;
                largest_area = area;
            }
        }
        return std::optional(largest);
    }
    else
    {
        return std::nullopt;
    }
}
