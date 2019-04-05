#include "ai/hl/stp/evaluation/enemy_threat.h"

#include <deque>

#include "ai/hl/stp/evaluation/intercept.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/evaluation/team.h"
#include "ai/world/world.h"
#include "geom/util.h"
#include "shared/constants.h"

std::optional<std::pair<int, std::optional<Robot>>> Evaluation::getNumPassesToRobot(
    const Robot& initial_passer, const Robot& final_receiver, const Team& team,
    const std::vector<Robot>& other_robots)
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
    std::vector<Robot> unvisited_robots = team.getAllRobots();
    // Remove the robot already in the current_passers since it starts off as visited
    unvisited_robots.erase(
        std::remove(unvisited_robots.begin(), unvisited_robots.end(), initial_passer),
        unvisited_robots.end());

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
    // The pass_num starts at one since the 0-th pass would be when the robot already
    // has the ball
    for (int pass_num = 1; pass_num < team.numRobots() && !current_passers.empty() &&
                           !unvisited_robots.empty();
         pass_num++)
    {
        // A comparator for the map below that stores passer and receiver robots.
        // This comparator is necessary for the Robot class to be used as a key in the
        // map. This comparator compares robots by ID.
        // See
        // https://stackoverflow.com/questions/6573225/what-requirements-must-stdmap-key-classes-meet-to-be-valid-keys
        // and
        // https://stackoverflow.com/questions/5733254/how-can-i-create-my-own-comparator-for-a-map
        //
        // This "custom" comparator is defined here rather than in the Robot class
        // as the '<' operator because there are many possible ways to order robots,
        // so it doesn't really make sense to define a single "correct" way in the
        // class, so we just define it as we need it here.
        struct cmpRobotByID
        {
            bool operator()(const Robot& r1, const Robot& r2) const
            {
                return r1.id() < r2.id();
            }
        };

        // Store a map of robots that can receive the ball, and the list of all robots
        // that could pass to them
        std::map<Robot, std::vector<Robot>, cmpRobotByID> receiver_passer_pairs;

        // For each of the current passers, check which unvisited robots they can
        // pass to
        for (const auto& current_passer : current_passers)
        {
            // Check if the current passer could pass to each unvisited robot
            for (const auto& unvisited_robot : unvisited_robots)
            {
                // Create a vector of obstacles that includes all robots except the
                // current passer and unvisited robot (aka the receiver)
                std::vector<Robot> obstacles = team.getAllRobots();
                obstacles.erase(
                    std::remove(obstacles.begin(), obstacles.end(), current_passer),
                    obstacles.end());
                obstacles.erase(
                    std::remove(obstacles.begin(), obstacles.end(), unvisited_robot),
                    obstacles.end());
                obstacles.insert(obstacles.end(), other_robots.begin(),
                                 other_robots.end());

                // Check if the pass from the current passer to the unvisited robot
                // would be blocked by any obstacle (other robot)
                bool pass_blocked = std::any_of(
                    obstacles.begin(), obstacles.end(),
                    [current_passer, unvisited_robot](const Robot& obstacle) {
                        return intersects(
                            Circle(obstacle.position(), ROBOT_MAX_RADIUS_METERS),
                            Segment(current_passer.position(),
                                    unvisited_robot.position()));
                    });

                if (!pass_blocked)
                {
                    if (receiver_passer_pairs.count(unvisited_robot) > 0)
                    {
                        // This unvisited_robot already exists in the map and can
                        // already be passed to by another robot. We add the passer
                        // to the list of possible passers for this robot
                        receiver_passer_pairs.at(unvisited_robot)
                            .emplace_back(current_passer);
                    }
                    else
                    {
                        // This unvisited robot does not exist in the map. Create a
                        // new entry to track this receiver and add the passer
                        receiver_passer_pairs.insert(std::make_pair(
                            unvisited_robot, std::vector<Robot>{current_passer}));
                    }
                }
            }
        }

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
