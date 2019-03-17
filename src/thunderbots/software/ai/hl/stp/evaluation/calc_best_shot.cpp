#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include "geom/util.h"

namespace Evaluation
{
    std::pair<Point, Angle> calcBestShotOnEnemyGoal(const Field &f,
                                                    const std::vector<Point> &obstacles,
                                                    const Point &p, double radius)
    {
        // Calculate the location of goalpost then use angleSweepCircle function to get
        // the pair
        const Point p1 = f.enemyGoalpostNeg();
        const Point p2 = f.enemyGoalpostPos();
        return angleSweepCircles(p, p1, p2, obstacles, radius);
    }

    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(
        const Field &f, const std::vector<Point> &obstacles, const Point &p,
        double radius)
    {
        const Point p1 = f.enemyGoalpostNeg();
        const Point p2 = f.enemyGoalpostPos();
        return angleSweepCirclesAll(p, p1, p2, obstacles, radius);
    }

    std::pair<Point, Angle> calcBestShotOnEnemyGoal(const World &world,
                                                    const Point &point, double radius)
    {
        std::vector<Point> obstacles;
        const Team &enemy    = world.enemyTeam();
        const Team &friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        // create a vector of points for all the robots except the shooting one
        for (const Robot &i : enemy.getAllRobots())
        {
            obstacles.emplace_back(i.position());
        }
        for (const Robot &fpl : friendly.getAllRobots())
        {
            if (fpl.position() == point)
            {
                continue;
            }
            obstacles.emplace_back(fpl.position());
        }
        std::pair<Point, Angle> best_shot =
            calcBestShotOnEnemyGoal(world.field(), obstacles, point, radius);
        // if there is no good shot at least make the
        // target within the goal area
        if (best_shot.second == Angle::zero())
        {
            Point temp      = world.field().enemyGoal();
            best_shot.first = temp;
        }
        return best_shot;
    }

    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(const World &world,
                                                                    const Point &point,
                                                                    double radius)
    {
        std::vector<Point> obstacles;
        const Team &enemy    = world.enemyTeam();
        const Team &friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        for (const Robot &i : enemy.getAllRobots())
        {
            obstacles.push_back(i.position());
        }
        for (const Robot &fpl : friendly.getAllRobots())
        {
            if (fpl.position() == point)
            {
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        return calcBestShotOnEnemyGoalAll(world.field(), obstacles, point, radius);
    }

}  // namespace Evaluation
