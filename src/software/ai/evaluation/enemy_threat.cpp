#include "software/ai/evaluation/enemy_threat.h"

#include <deque>
#include <boost/geometry/algorithms/detail/relate/result.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/geometry/strategies/normalize.hpp>
#include <boost/geometry/util/math.hpp>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/possession.h"
#include "software/geom/algorithms/intersects.h"
#include "software/logger/logger.h"
#include "software/world/team.h"

std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> findAllReceiverPasserPairs(
    const std::vector<Robot> &possible_passers,
    const std::vector<Robot> &possible_receivers, const std::vector<Robot> &all_robots)
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
                            [passer, receiver](const Robot &obstacle)
                            {
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

std::optional<std::pair<int, std::optional<Robot>>> getNumPassesToRobot(
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
    // TODO: possibly re-enable using friendly robots as obstacles if we can find a way to
    // stop defenders from oscillating between positions See
    // https://github.com/UBC-Thunderbots/Software/issues/642
    // all_robots.insert(all_robots.end(), other_robots.begin(), other_robots.end());

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
    for (unsigned pass_num = 1; pass_num < passing_team.numRobots() &&
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
            auto closest_passer = Team::getNearestRobot(
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

void sortThreatsInDecreasingOrder(std::vector<std::pair<EnemyThreat, float>> threats)
{
    // A lambda function that implements the '<' operator for the EnemyThreat struct
    // so it can be sorted. Lower threats are "less than" higher threats.
    auto enemyThreatLessThanComparator = [](const std::pair<EnemyThreat, float> &a, const std::pair<EnemyThreat, float> &b)
    {
        // Robots with the ball are more threatening than robots without the ball, and
        // robots with the ball are the most threatening since they can shoot or move
        // the ball towards our net
        if (a.first.has_ball && !b.first.has_ball)
        {
            return false;
        }
        else if (!a.first.has_ball && b.first.has_ball)
        {
            return true;
        }
        if (a.second>b.second)
        {
            return false;
        }
        else if (a.second<b.second)
        {
            return true;
        }
        else
        {
            //get closer distance
            if (a.first.best_shot_angle>b.first.best_shot_angle)
            {
                return false;
            }
            else
            {
                return true;
            }
        }


    };

    // Sort threats from highest threat to lowest threat
    // Use reverse iterators to sort the vector in descending order
    std::sort(threats.rbegin(), threats.rend(), enemyThreatLessThanComparator);

}

float get_threat_score(const EnemyThreat& enemy, const Ball& ball)
{
    //Pure shot angle
    Angle goal_angle = enemy.goal_angle;
    Angle mid = Angle::fromRadians(boost::math::float_constants::pi/4);
    int k = 5;
    float S_geo = 1/(1+exp(-k*(goal_angle.toRadians()-mid.toRadians())));

    //Distance from ball normalized
    float distance_ball = (enemy.robot.position()-ball.position()).length();
    float S_pos = expf(-0.8f *distance_ball);

    //Orientation
    Vector to_goal = Point(3,0) - enemy.robot.position();
    float facing_direction = to_goal.normalize().dot(enemy.robot.velocity().normalize());
    float S_ori = std::max(static_cast<float>(0.0),facing_direction);

    //Visibility
    float block_frac = enemy.best_shot_angle->toRadians()/enemy.goal_angle.toRadians();
    float S_vis = 1.0f - block_frac;

    //predictive motion
    float vel_dot = enemy.robot.velocity().normalize().dot(to_goal.normalize());
    float S_pred = std::max(0.0f,vel_dot)*1.0f+0.2f*enemy.robot.velocity().length();

    return 0.3f*S_geo + 0.25f*S_pos + 0.15f*S_ori + 0.2f*S_vis + 0.1f*S_pred;
}

std::vector<EnemyThreat> getAllEnemyThreats(const Field &field, const Team &friendly_team,
                                            Team enemy_team, const Ball &ball,
                                            bool include_goalie)
{
    std::vector<std::pair<EnemyThreat, float>> robot_to_threat;
    if (!include_goalie && enemy_team.getGoalieId())
    {
        enemy_team.removeRobotWithId(*enemy_team.getGoalieId());
    }

    for (const auto &robot : enemy_team.getAllRobots())
    {
        bool has_ball = robot.isNearDribbler(ball.position());

        // Get the angle from the robot to each friendly goalpost, then find the
        // difference between these angles to get the goal_angle for the robot
        auto friendly_goalpost_angle_1 =
            (field.friendlyGoalpostPos() - robot.position()).orientation();
        auto friendly_goalpost_angle_2 =
            (field.friendlyGoalpostNeg() - robot.position()).orientation();
        Angle goal_angle = friendly_goalpost_angle_1.minDiff(friendly_goalpost_angle_2);

        std::optional<Angle> best_shot_angle  = std::nullopt;
        std::optional<Point> best_shot_target = std::nullopt;
        auto best_shot_data =
            calcBestShotOnGoal(field, friendly_team, enemy_team, robot.position(),
                               TeamType::FRIENDLY, {robot});
        if (best_shot_data)
        {
            best_shot_angle  = best_shot_data->getOpenAngle();
            best_shot_target = best_shot_data->getPointToShootAt();
        }

        // Set default values. If the robot can't be passed to we set the number of passes
        // to the size of the enemy team so it is the largest reasonable value, and the
        // passer to be an empty optional
        int num_passes              = static_cast<int>(enemy_team.numRobots());
        std::optional<Robot> passer = std::nullopt;
        auto robot_with_effective_possession =
            getRobotWithEffectiveBallPossession(enemy_team, ball, field);
        if (robot_with_effective_possession)
        {
            auto pass_data = getNumPassesToRobot(robot_with_effective_possession.value(),
                                                 robot, enemy_team, friendly_team);
            if (pass_data)
            {
                num_passes = pass_data->first;
                passer     = pass_data->second;
            }
        }

        EnemyThreat threat{robot,     has_ball,         goal_angle,
                           best_shot_angle, best_shot_target, num_passes,
                           passer};

        robot_to_threat.emplace_back(threat,get_threat_score(threat,ball));
    }

    // Sort the threats so the "most threatening threat" is first in the vector, and the
    // "least threatening threat" is last in the vector
    sortThreatsInDecreasingOrder(robot_to_threat);
    std::vector<EnemyThreat> threats;
    threats.reserve(robot_to_threat.size());
    for (const auto p: robot_to_threat)
    {
        threats.push_back(p.first);
    }
    return threats;
}
