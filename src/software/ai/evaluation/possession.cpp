#include "software/ai/evaluation/possession.h"

#include <algorithm>

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/geom/algorithms/distance.h"

std::optional<Robot> getRobotWithEffectiveBallPossession(
    const Team& team, const Ball& ball, const Field& field,
    std::optional<RobotId> previous_baller, const Duration& switch_margin)
{
    if (team.numRobots() == 0)
    {
        return std::nullopt;
    }

    const auto& robots = team.getAllRobots();
    auto best_intercept = findBestInterceptForBall(ball, field, robots.at(0));
    auto baller_robot   = robots.at(0);
    std::optional<Duration> previous_baller_time;
    if (previous_baller && robots.at(0).id() == *previous_baller && best_intercept)
    {
        previous_baller_time = best_intercept->second;
    }

    // Find the robot that can intercept the ball the quickest
    for (size_t i = 1; i < robots.size(); ++i)
    {
        const auto& robot = robots[i];
        auto intercept    = findBestInterceptForBall(ball, field, robot);
        if (previous_baller && robot.id() == *previous_baller && intercept)
        {
            previous_baller_time = intercept->second;
        }
        if (!best_intercept || (intercept && intercept->second < best_intercept->second))
        {
            best_intercept = intercept;
            baller_robot   = robot;
        }
    }

    // Keep the previous baller unless another robot is clearly faster, so near-tied
    // intercept times do not flip the result between updates
    if (best_intercept && previous_baller_time &&
        *previous_baller_time <= best_intercept->second + switch_margin)
    {
        auto previous = std::find_if(robots.begin(), robots.end(),
                                     [&](const Robot& robot)
                                     { return robot.id() == *previous_baller; });
        return *previous;
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
