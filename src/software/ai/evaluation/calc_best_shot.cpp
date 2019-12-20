#include "software/ai/evaluation/calc_best_shot.h"

#include "software/geom/util.h"

namespace Evaluation
{
    std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                           const Point &goal_post_pos, const Point &p,
                                           const std::vector<Point> &obstacles,
                                           double radius)
    {
        // Use angleSweepCircle function to get the pair
        return angleSweepCircles(p, goal_post_neg, goal_post_pos, obstacles, radius);
    }

    std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                           const Team &enemy_team, const Point &point,
                                           bool shoot_on_enemy_goal, double radius,
                                           const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Point> obstacles;
        for (const Robot &enemy_robot : enemy_team.getAllRobots())
        {
            // Only add the robot to the obstacles if it is not ignored
            if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                           enemy_robot) == 0)
            {
                obstacles.emplace_back(enemy_robot.position());
            }
        }
        for (const Robot &friendly_robot : friendly_team.getAllRobots())
        {
            // Only add the robot to the obstacles if it is not ignored
            if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                           friendly_robot) == 0)
            {
                obstacles.emplace_back(friendly_robot.position());
            }
        }

        std::optional<Shot> best_shot;

        // Calculate the best_shot based on what goal we're shooting at
        if (shoot_on_enemy_goal)
        {
            best_shot =
                calcBestShotOnGoal(field.enemyGoalpostNeg(), field.enemyGoalpostPos(),
                                   point, obstacles, radius);
        }
        else
        {
            best_shot =
                calcBestShotOnGoal(field.friendlyGoalpostPos(),
                                   field.friendlyGoalpostNeg(), point, obstacles, radius);
        }

        return best_shot;
    }

    std::optional<Shot> calcBestShotOnEnemyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Robot &robot, double radius, const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnEnemyGoal(field, friendly_team, enemy_team, robot.position(),
                                       radius, all_robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnEnemyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, true,
                                  radius, robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnFriendlyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Robot &robot, double radius, const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnFriendlyGoal(field, friendly_team, enemy_team,
                                          robot.position(), radius, all_robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnFriendlyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, false,
                                  radius, robots_to_ignore);
    }

    double calcShotOpenFriendlyNetPercentage(const Field &field, const Point &shot_origin,
                                             const Shot &shot)
    {
        Angle goal_angle = acuteVertexAngle(field.friendlyGoalpostPos(), shot_origin,
                                            field.friendlyGoalpostNeg())
                               .abs();
        return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
    }

    double calcShotOpenEnemyNetPercentage(const Field &field, const Point &shot_origin,
                                          const Shot &shot)
    {
        Angle goal_angle = acuteVertexAngle(field.enemyGoalpostPos(), shot_origin,
                                            field.enemyGoalpostNeg())
                               .abs();
        return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
    }
}  // namespace Evaluation
