#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include "geom/util.h"
#include "shared/constants.h"

namespace Evaluation
{
    std::pair<Point, Angle> calcBestShot(const Field& f,
                                         const std::vector<Point>& obstacles,
                                         const Point& p, const double radius)
    {
        // Calculate the location of goalpost then use angleSweepCircle function to get
        // the pair
        const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
        return angleSweepCircles(p, p1, p2, obstacles, radius * ROBOT_MAX_RADIUS_METERS);
    }

    std::vector<std::pair<Point, Angle>> calcBestShotAll(
        const Field& f, const std::vector<Point>& obstacles, const Point& p,
        const double radius)
    {
        const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
        return angleSweepCirclesAll(p, p1, p2, obstacles,
                                    radius * ROBOT_MAX_RADIUS_METERS);
    }

    std::pair<Point, Angle> calcBestShot(const World& world, const Robot& robot,
                                         double radius)
    {
        std::vector<Point> obstacles;
        const Team& enemy    = world.enemyTeam();
        const Team& friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        // create a vector of points for all the robots except the shooting one
        for (const Robot& i : enemy.getAllRobots())
        {
            obstacles.emplace_back(i.position());
        }
        for (const Robot& fpl : friendly.getAllRobots())
        {
            if (fpl == robot)
            {
                continue;
            }
            obstacles.emplace_back(fpl.position());
        }
        std::pair<Point, Angle> best_shot =
            calcBestShot(world.field(), obstacles, robot.position(), radius);
        // if there is no good shot at least make the
        // target within the goal area
        if (best_shot.second <= Angle::zero())
        {
            Point temp      = Point(world.field().length() / 2.0, 0.0);
            best_shot.first = temp;
        }
        return best_shot;
    }

    std::vector<std::pair<Point, Angle>> calcBestShotAll(const World& world,
                                                         const Robot& robot,
                                                         double radius)
    {
        std::vector<Point> obstacles;
        const Team& enemy    = world.enemyTeam();
        const Team& friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        for (const Robot& i : enemy.getAllRobots())
        {
            obstacles.push_back(i.position());
        }
        for (const Robot& fpl : friendly.getAllRobots())
        {
            if (fpl == robot)
            {
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        return calcBestShotAll(world.field(), obstacles, robot.position(), radius);
    }

}  // namespace Evaluation
