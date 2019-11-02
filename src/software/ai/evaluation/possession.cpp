#include "software/ai/evaluation/possession.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/evaluation/team.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/ball.h"
#include "software/world/field.h"

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

            unsigned i = 0;

            // Check that the robot has had possession of the ball recently.
            while (
                i < robot_history_timestamps.size() &&
                robot.lastUpdateTimestamp() - robot_history_timestamps[i] <=
                    Duration::fromSeconds(Util::DynamicParameters->getEvaluationConfig()
                                              ->getPossessionConfig()
                                              ->PossessionBufferTimeSeconds()
                                              ->value()))
            {
                std::optional<bool> robot_has_possession =
                    robotHasPossession(world.ball(), robot, robot_history_timestamps[i]);
                if (robot_has_possession.has_value() && *robot_has_possession)
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
                   Duration::fromSeconds(Util::DynamicParameters->getEvaluationConfig()
                                             ->getPossessionConfig()
                                             ->PassBufferTimeSeconds()
                                             ->value()))
            {
                std::optional<bool> robot_being_passed_to =
                    robotBeingPassedTo(world, robot, robot_history_timestamps[i]);
                if (robot_being_passed_to.has_value() && *robot_being_passed_to)
                    return true;
                i++;
            }
        }

        return false;
    }
}  // namespace Evaluation
