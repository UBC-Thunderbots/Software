#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include "geom/util.h"

namespace Evaluation
{
    std::pair<Point, Angle> calcBestShotOnGoal(const Point &goal_post_neg,
                                               const Point &goal_post_pos, const Point &p,
                                               const std::vector<Point> &obstacles,
                                               double radius)
    {
        // Use angleSweepCircle function to get the pair
        return angleSweepCircles(p, goal_post_neg, goal_post_pos, obstacles, radius);
    }

    std::pair<Point, Angle> calcBestShotOnGoal(const World &world, const Point &point,
                                               double radius,
                                               const std::vector<Robot> &robots_to_ignore,
                                               bool shoot_on_enemy_goal)
    {
        std::vector<Point> obstacles;
        for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
        {
            // Only add the robot to the obstacles if it is not ignored
            if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                           enemy_robot) == 0)
            {
                obstacles.emplace_back(enemy_robot.position());
            }
        }
        for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
        {
            // Only add the robot to the obstacles if it is not ignored
            if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                           friendly_robot) == 0)
            {
                obstacles.emplace_back(friendly_robot.position());
            }
        }

        std::pair<Point, Angle> best_shot;

        // Calculate the best_shot based on what goal we're shooting at
        if (shoot_on_enemy_goal)
        {
            best_shot = calcBestShotOnGoal(world.field().enemyGoalpostNeg(),
                                           world.field().enemyGoalpostPos(), point,
                                           obstacles, radius);
            // If no shot is found, at least set the target to the goal
            if (best_shot.second == Angle::zero())
            {
                best_shot.first = world.field().enemyGoal();
            }
        }
        else
        {
            best_shot = calcBestShotOnGoal(world.field().friendlyGoalpostPos(),
                                           world.field().friendlyGoalpostNeg(), point,
                                           obstacles, radius);
            // If not shot is found, at least set the target to the goal
            if (best_shot.second == Angle::zero())
            {
                best_shot.first = world.field().friendlyGoal();
            }
        }

        return best_shot;
    }

    std::pair<Point, Angle> calcBestShotOnEnemyGoal(
        const World &world, const Robot &robot, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnEnemyGoal(world, robot.position(), radius,
                                       all_robots_to_ignore);
    }

    std::pair<Point, Angle> calcBestShotOnEnemyGoal(
        const World &world, const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(world, shot_origin, radius, robots_to_ignore, true);
    }

    std::pair<Point, Angle> calcBestShotOnFriendlyGoal(
        const World &world, const Robot &robot, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnFriendlyGoal(world, robot.position(), radius,
                                          all_robots_to_ignore);
    }

    std::pair<Point, Angle> calcBestShotOnFriendlyGoal(
        const World &world, const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(world, shot_origin, radius, robots_to_ignore, false);
    }
}  // namespace Evaluation
