#include "ai/world/world.h"

namespace Evaluation
{
    /**
     * Returns the robot that either has the ball, or is the closest to having it (and
     * therefore has the most "presence" over the ball)
     *
     * @param team The team containing the robots to check for possession
     * @param ball the Ball
     * @param field The Field being played on
     * @return the robot that either has the ball, or is the closest to having it. If the
     * team has no robots, std::nullopt is returned
     */
    std::optional<Robot> getRobotWithEffectiveBallPossession(const Team &team,
                                                             const Ball &ball,
                                                             const Field &field);
    /**
     * Return true if the provided team has possession of the ball. A team is considered
     * to have possession if any robot on the team has had possession within the last
     * "TIME BUFFER" seconds, so that we can account for small transition periods like
     * passing where a robot might not directly have the ball at that moment in time.
     *
     * @param team The team containing the robots to check for possession
     * @param ball The ball
     * @return True if the team has possession, false otherwise
     */
    bool teamHasPossession(const World& world, const Team &team);

    bool teamPassInProgress(const World& world, const Team& team);
}  // namespace Evaluation
