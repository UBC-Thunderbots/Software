#include <optional>
#include <vector>

#include "ai/world/world.h"


namespace Evaluation
{
    /**
     * Returns a map of robots that can receive a pass from a passer, and the list of
     * passers that could pass to each receiver.
     *
     * @param possible_passers Robots that could pass the ball
     * @param possible_receivers Robots that could receive the ball
     * @param all_robots All robots on the field, both friendly and enemy, including the
     * possible passers and receivers
     * @return A map of robots that can receive a pass from a passer, and the list of
     * passers that could pass to each receiver
     */
    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> findAllReceiverPasserPairs(
        const std::vector<Robot> &possible_passers,
        const std::vector<Robot> &possible_receivers,
        const std::vector<Robot> &all_robots);

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
     * @param passing_team The team the passer and receiver robots are a part of
     * @param other_team The other team (the enemy of the passing team)
     *
     * @return a pair containing the number of passes it will take for the passer robot to
     * pass the ball to the receiver robot, and the intermediate robot the receiver will
     * receive the pass from
     */
    std::optional<std::pair<int, std::optional<Robot>>> getNumPassesToRobot(
        const Robot &initial_passer, const Robot &final_receiver,
        const Team &passing_team, const Team &other_team);
}  // namespace Evaluation
