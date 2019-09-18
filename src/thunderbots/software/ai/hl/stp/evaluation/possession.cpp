#include "ai/hl/stp/evaluation/possession.h"

#include <shared/constants.h>

#include "ai/hl/stp/evaluation/intercept.h"
#include "ai/hl/stp/evaluation/team.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/world.h"
#include "robot.h"

namespace Evaluation
{
    std::optional<Robot> getRobotWithEffectiveBallPossession(const Team &team,
                                                             const Ball &ball,
                                                             const Field &field)
    {
        if (team.numRobots() == 0)
        {
            return std::nullopt;
        }

        auto best_intercept =
            Evaluation::findBestInterceptForBall(ball, field, team.getAllRobots().at(0));
        auto baller_robot = team.getAllRobots().at(0);

        // Find the robot that can intercept the ball the quickest
        for (const auto &robot : team.getAllRobots())
        {
            auto intercept = Evaluation::findBestInterceptForBall(ball, field, robot);
            if (!best_intercept ||
                (intercept && intercept->second < best_intercept->second))
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
            return Evaluation::nearestRobot(team, ball.position());
        }
    }

    bool teamHasPossession(const World &world, const Team &team)
    {
        for (const Robot &robot : team.getAllRobots())
        {
            std::vector<Timestamp> robot_history_timestamps =
                robot.getPreviousTimestamps();

            int i = 0;

            // Check that the robot has had possession of the ball recently.
            while (i < robot_history_timestamps.size() &&
                   robot.lastUpdateTimestamp() - robot_history_timestamps[i] <=
                       Duration::fromSeconds(POSSESSION_BUFFER_TIME_IN_SECONDS))
            {
                if (robotHasPossession(world.ball(), robot, robot_history_timestamps[i]))
                    return true;
                i++;
            }
        }

        return false;
    }

    bool teamPassInProgress(const World &world, const Team &team)
    {
        for (const Robot &robot : team.getAllRobots())
        {
            std::vector<Timestamp> robot_history_timestamps =
                robot.getPreviousTimestamps();

            int i = 0;

            // Check that the robot has had possession of the ball recently.
            while (robot.lastUpdateTimestamp() - robot_history_timestamps[i] <
                   Duration::fromSeconds(PASS_BUFFER_TIME_IN_SECONDS))
            {
                if (robotBeingPassedTo(world, robot, robot_history_timestamps[i]))
                    return true;
                i++;
            }
        }

        return false;
    }
}  // namespace Evaluation
