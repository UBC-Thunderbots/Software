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
        if (!best_intercept ||
            (intercept && intercept.value().duration < best_intercept.value().duration))
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
