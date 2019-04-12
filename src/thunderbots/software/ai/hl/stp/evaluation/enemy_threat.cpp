#include "ai/hl/stp/evaluation/enemy_threat.h"

#include <deque>

#include "ai/hl/stp/evaluation/intercept.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/evaluation/team.h"
#include "ai/world/world.h"
#include "geom/util.h"
#include "shared/constants.h"

std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID>
Evaluation::findAllReceiverPasserPairs(const std::vector<Robot> &possible_passers,
                                       const std::vector<Robot> &possible_receivers,
                                       const std::vector<Robot> &all_robots)
{
    // Store a map of robots that can receive the ball, and the list of all robots
    // that could pass to them. The custom comparator is necessary to use the Robot
    // class as a key in the map
    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> receiver_passer_pairs;

    // For each of the passers, check which robots they could pass to
    for (const auto &passer : possible_passers)
    {
        for (const auto &receiver : possible_receivers)
        {
            // Create a vector of obstacles that includes all robots except the
            // current passer and receiver
            std::vector<Robot> obstacles = all_robots;
            obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), passer),
                            obstacles.end());
            obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), receiver),
                            obstacles.end());

            // Check if the pass from the passer to the receiver would be blocked by any
            // robots
            bool pass_blocked =
                std::any_of(obstacles.begin(), obstacles.end(),
                            [passer, receiver](const Robot &obstacle) {
                                return intersects(
                                    Circle(obstacle.position(), ROBOT_MAX_RADIUS_METERS),
                                    Segment(passer.position(), receiver.position()));
                            });

            if (!pass_blocked)
            {
                if (receiver_passer_pairs.count(receiver) > 0)
                {
                    // This receiver already exists in the map and can
                    // already be passed to by another robot. We add the passer
                    // to the list of possible passers for this robot
                    receiver_passer_pairs.at(receiver).emplace_back(passer);
                }
                else
                {
                    // This receiver robot does not exist in the map. Create a
                    // new entry to track this receiver and add the passer
                    receiver_passer_pairs.insert(
                        std::make_pair(receiver, std::vector<Robot>{passer}));
                }
            }
        }
    }

    return receiver_passer_pairs;
}

std::optional<std::pair<int, std::optional<Robot>>> Evaluation::getNumPassesToRobot(
    const Robot &initial_passer, const Robot &final_receiver, const Team &passing_team,
    const Team &other_team)
{
    if (initial_passer == final_receiver)
    {
        return std::make_pair(0, std::nullopt);
    }

    // We calculate the minimum number of passes it would take for the initial_passer
    // robot to pass the ball to the final_receiver, assuming both robots are on the given
    // team
    //
    // This algorithm essentially treats the team of robots like a Directed Acyclic
    // Graph and "traverses" the graph to find the shortest path to the robot

    // The robots that could have the ball and make a pass at each iteration. They
    // are on the "frontier" of the graph search
    std::vector<Robot> current_passers{initial_passer};
    // The remaining robots we haven't checked yet
    std::vector<Robot> unvisited_robots = passing_team.getAllRobots();
    // Remove the initial passer since we already start off visiting it, and don't need
    // to again
    unvisited_robots.erase(
        std::remove(unvisited_robots.begin(), unvisited_robots.end(), initial_passer),
        unvisited_robots.end());
    std::vector<Robot> all_robots   = passing_team.getAllRobots();
    std::vector<Robot> other_robots = other_team.getAllRobots();
    all_robots.insert(all_robots.end(), other_robots.begin(), other_robots.end());

    // On each iteration, check what robots can be passed to. These receivers will
    // become the passers on the next iteration. This is like expanding the frontier
    // of the graph
    //
    // We continue to iterate and check greater numbers of passes until one of the
    // following cases:
    // * There are no more passers at the end of the iteration. This means no more
    //   unvisited robots can be passed to
    // * There are no more unvisited robots
    // * We have iterated up to the size of the team. This is a fallback case to
    //   prevent any infinite loops, just in case
    //
    // We already check the case where the passer and receiver are the same. If this was
    // the case, 0 passes would be required. Since that case is already checked, when we
    // start the loop we are checking for the possibility of the receiver getting the ball
    // in 1 pass. This is why pass_num starts at 1.
    for (int pass_num = 1; pass_num < passing_team.numRobots() &&
                           !current_passers.empty() && !unvisited_robots.empty();
         pass_num++)
    {
        std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> receiver_passer_pairs =
            findAllReceiverPasserPairs(current_passers, unvisited_robots, all_robots);

        // If the robot we are looking for is a receiver, return the number of
        // passes and the passer to this robot
        if (receiver_passer_pairs.count(final_receiver) > 0)
        {
            // If there are multiple robots that can pass to the robot, we assume
            // it will receive the ball from the closest one since this is more
            // likely
            auto closest_passer = Evaluation::nearestRobot(
                receiver_passer_pairs.at(final_receiver), final_receiver.position());
            return std::make_pair(pass_num, closest_passer);
        }

        // Create a list of all robots that could receive the ball this iteration
        std::vector<Robot> all_receivers;
        for (auto it = receiver_passer_pairs.begin(); it != receiver_passer_pairs.end();
             it++)
        {
            all_receivers.emplace_back(it->first);
        }

        // All robots that could have received the ball now become passers
        current_passers = all_receivers;

        // Remove any receivers from the unvisited robots, since they have
        // now been visited
        unvisited_robots.erase(remove_if(unvisited_robots.begin(), unvisited_robots.end(),
                                         [&](auto x) {
                                             return find(all_receivers.begin(),
                                                         all_receivers.end(),
                                                         x) != all_receivers.end();
                                         }),
                               unvisited_robots.end());
    }

    // If we have checked all the robots we can and still haven't found the robot we
    // are looking for, it must be blocked and unable to be passed to in the current
    // state. Therefore, we return an std::nullopt
    return std::nullopt;
}
