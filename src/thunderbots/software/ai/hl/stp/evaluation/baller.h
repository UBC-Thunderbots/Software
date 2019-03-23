#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/team.h"

namespace Evaluation
{
    /**
     * Returns the robot that either has the ball, or is the closest to having it (and
     * therefore has the most "presence" over the ball)
     *
     * @param team The team to get the baller from
     * @param ball the Ball
     * @param field The Field being played on
     * @return the robot that either has the ball, or is the closest to having it. If the
     * team has no robots, std::nullopt is returned
     */
    std::optional<Robot> getTeamBaller(const Team& team, const Ball& ball,
                                       const Field& field);
}  // namespace Evaluation
