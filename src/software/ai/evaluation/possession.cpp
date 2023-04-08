#include "software/ai/evaluation/possession.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"

std::optional<Robot> getRobotWithEffectiveBallPossession(const Team &team,
                                                         const Ball &ball,
                                                         const Field &field)
{
    if (team.numRobots() == 0)
    {
        return std::nullopt;
    }

    auto best_intercept =
        findBestInterceptForBall(ball, field, team.getAllRobots().at(0));
    auto baller_robot = team.getAllRobots().at(0);

    // Find the robot that can intercept the ball the quickest
    for (const auto &robot : team.getAllRobots())
    {
        auto intercept = findBestInterceptForBall(ball, field, robot);
        if (!best_intercept || (intercept && intercept->second < best_intercept->second))
        {
            best_intercept = intercept;
            baller_robot   = robot;
        }
    }

    // Return the robot that can intercept the ball the fastest within the field. If
    // no robot is able to intercept the ball within the field, return the closest
    // robot to the ball
    if (best_intercept)
    {
        return baller_robot;
    }
    else
    {
        return team.getNearestRobot(ball.position());
    }
}

std::optional<Team> getTeamWithEffectiveBallPossession(const Team &friendly_team,
                                                       const Team &enemy_team,
                                                       const Ball &ball,
                                                       const Field &field)
{
    if (friendly_team.numRobots() == 0)
    {
        return enemy_team;
    }
    if (enemy_team.numRobots() == 0)
    {
        return friendly_team;
    }
    if (friendly_team.numRobots() == 0 && enemy_team.numRobots() == 0)
    {
        return std::nullopt;
    }

    // Find the robot that can intercept the ball the quickest
    Team team_with_possession;
    auto best_intercept =
        findBestInterceptForBall(ball, field, friendly_team.getAllRobots().at(0));
    for (const auto &robot : friendly_team.getAllRobots())
    {
        auto intercept = findBestInterceptForBall(ball, field, robot);
        if (!best_intercept || (intercept && intercept->second < best_intercept->second))
        {
            best_intercept       = intercept;
            team_with_possession = friendly_team;
        }
    }

    for (const auto &robot : enemy_team.getAllRobots())
    {
        auto intercept = findBestInterceptForBall(ball, field, robot);
        if (!best_intercept || (intercept && intercept->second < best_intercept->second))
        {
            best_intercept       = intercept;
            team_with_possession = enemy_team;
        }
    }

    if (best_intercept)
    {
        return team_with_possession;
    }
    else
    {
        auto closest_friendly_robot = *friendly_team.getNearestRobot(ball.position());
        auto closest_enemy_robot    = *enemy_team.getNearestRobot(ball.position());

        if ((ball.position() - closest_friendly_robot.position()).length() <
            (ball.position() - closest_enemy_robot.position()).length())
        {
            return friendly_team;
        }
        return enemy_team;
    }
}
