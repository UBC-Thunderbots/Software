#include <optional>
#include <vector>

#include "ai/world/world.h"


namespace Evaluation
{
    /**
     * Returns how many passes it would take for the given passer to pass the ball to the
     * receiver so the receiver gains possession of the ball, and returns the intermediate
     * passer the receiver is most likely to receive the ball from.
     *
     * If the passing and receiving robot are the same, the number of passes is 0 and the
     * passer value is an std::nullopt. If the receiver cannot be passed to at all
     * (all passing routes are blocked), then an std::nullopt is returned
     *
     * @param initial_passer The robot the passes start from
     * @param final_receiver The robot trying to be passed to
     * @param team The team the robots are a part of
     * @param other_robots Any other robots (in addition to the given team) that may be on
     * the field acting as obstacles
     *
     * @return a pair containing the number of passes it will take for the passer robot to
     * pass the ball to the receiver robot, and the intermediate robot the receiver will
     * receive the pass from
     */
    std::optional<std::pair<int, std::optional<Robot>>> getNumPassesToRobot(
        const Robot &initial_passer, const Robot &final_receiver, const Team &team,
        const std::vector<Robot> &other_robots);
}  // namespace Evaluation
