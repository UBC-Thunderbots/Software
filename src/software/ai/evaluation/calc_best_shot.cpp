#include "software/ai/evaluation/calc_best_shot.h"

#include "software/ai/evaluation/calc_best_shot_impl.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/multiple_segments.h"
#include "software/geom/algorithms/projection.h"
#include "software/ai/evaluation/calc_best_shot_exp.h"

std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles)
{
    // Use shot evaluation function to get the best Shot
    std::vector<Circle> obs;
    for (Robot robot : robot_obstacles)
    {
        obs.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
    }
    return calcMostOpenDirectionFromCircleObstacles(shot_origin, goal_post, obs);
}

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore,
                                       double radius)
{
    return calcBestShotOnGoalExp(field, friendly_team, enemy_team, shot_origin, goal, robots_to_ignore);
//    std::vector<Robot> obstacles;
//    for (const Robot &enemy_robot : enemy_team.getAllRobots())
//    {
//        // Only add the robot to the obstacles if it is not ignored
//        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
//            0)
//        {
//            obstacles.emplace_back(enemy_robot);
//        }
//    }
//    for (const Robot &friendly_robot : friendly_team.getAllRobots())
//    {
//        // Only add the robot to the obstacles if it is not ignored
//        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
//                       friendly_robot) == 0)
//        {
//            obstacles.emplace_back(friendly_robot);
//        }
//    }
//
//    // Calculate the best_shot based on what goal we're shooting at
//    if (goal == TeamType::FRIENDLY)
//    {
//        return calcBestShotOnGoal(
//            Segment(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos()),
//            shot_origin, obstacles);
//    }
//    else
//    {
//        return calcBestShotOnGoal(
//            Segment(field.enemyGoalpostNeg(), field.enemyGoalpostPos()), shot_origin,
//            obstacles);
//    }
}
