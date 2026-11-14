#include "software/ai/evaluation/enemy_threat.h"

#include <deque>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/possession.h"
#include "software/geom/algorithms/intersects.h"
#include "software/world/team.h"


std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> findAllReceiverPasserPairs(
    const std::vector<Robot>& possible_passers,
    const std::vector<Robot>& possible_receivers, const std::vector<Robot>& all_robots)
{
    // Store a map of robots that can receive the ball, and the list of all robots
    // that could pass to them. The custom comparator is necessary to use the Robot
    // class as a key in the map
    std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> receiver_passer_pairs;

    // For each of the passers, check which robots they could pass to
    for (const auto& passer : possible_passers)
    {
        for (const auto& receiver : possible_receivers)
        {
            const Segment pass(passer.position(), receiver.position());
            bool pass_blocked = std::any_of(
                all_robots.begin(), all_robots.end(),
                [&](const Robot& obstacle)
                {
                    return !(obstacle == passer) && !(obstacle == receiver) &&
                           intersects(
                               Circle(obstacle.position(), ROBOT_MAX_RADIUS_METERS),
                               pass);
                });

            if (!pass_blocked)
            {
                receiver_passer_pairs[receiver].emplace_back(passer);
            }
        }
    }

    return receiver_passer_pairs;
}

/**
 * Returns whether a robot at the given position could shoot on the friendly goal
 */
bool canShootOnFriendlyGoal(const Field& field, const Point& position)
{
    // Compare along the axis between the two goals so this doesn't depend on which
    // side of the field we are defending
    Vector towards_enemy_goal = field.enemyGoalCenter() - field.friendlyGoalCenter();
    Vector goal_to_position   = position - field.friendlyGoalCenter();

    // A robot level with the goal line sits between the goalposts rather than in front
    // of them, so require it to be at least its own radius ahead of the line before we
    // treat it as having anything to shoot at
    return goal_to_position.dot(towards_enemy_goal.normalize()) > ROBOT_MAX_RADIUS_METERS;
}

namespace
{
// Maps a robot to the number of passes needed to reach it and the robot it receives from
using NumPassesToRobotMap =
    std::map<Robot, std::pair<int, std::optional<Robot>>, Robot::cmpRobotByID>;

/**
 * Returns how many passes it would take for the initial passer to pass the ball to each
 * robot on its team, and the robot each of them would receive the ball from. Robots that
 * cannot be passed to at all are absent from the returned map.
 */
NumPassesToRobotMap getNumPassesToAllRobots(const Robot& initial_passer,
                                            const Team& passing_team,
                                            const Team& other_team)
{
    NumPassesToRobotMap num_passes_to_robot = {{initial_passer, {0, std::nullopt}}};

    // We calculate the minimum number of passes it would take for the initial_passer
    // robot to pass the ball to each other robot on the given team
    //
    // This algorithm essentially treats the team of robots like a Directed Acyclic
    // Graph and "traverses" the graph to find the shortest path to each robot

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
    const auto& all_robots = passing_team.getAllRobots();
    // TODO: possibly re-enable using friendly robots as obstacles if we can find a way to
    // stop defenders from oscillating between positions See
    // https://github.com/UBC-Thunderbots/Software/issues/642

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
    // The initial passer is already recorded as needing 0 passes, so pass_num starts at 1
    for (unsigned pass_num = 1; pass_num < passing_team.numRobots() &&
                                !current_passers.empty() && !unvisited_robots.empty();
         pass_num++)
    {
        std::map<Robot, std::vector<Robot>, Robot::cmpRobotByID> receiver_passer_pairs =
            findAllReceiverPasserPairs(current_passers, unvisited_robots, all_robots);

        // Create a list of all robots that could receive the ball this iteration
        std::vector<Robot> all_receivers;
        all_receivers.reserve(receiver_passer_pairs.size());
        for (const auto& [receiver, passers] : receiver_passer_pairs)
        {
            // If there are multiple robots that can pass to the robot, we assume it
            // will receive the ball from the closest one since this is more likely
            num_passes_to_robot.emplace(
                receiver,
                std::make_pair(static_cast<int>(pass_num),
                               Team::getNearestRobot(passers, receiver.position())));
            all_receivers.emplace_back(receiver);
        }

        // Remove any receivers from the unvisited robots, since they have
        // now been visited
        unvisited_robots.erase(remove_if(unvisited_robots.begin(), unvisited_robots.end(),
                                         [&](const Robot& x) {
                                             return find(all_receivers.begin(),
                                                         all_receivers.end(),
                                                         x) != all_receivers.end();
                                         }),
                               unvisited_robots.end());
        // Reuse the receiver vector as the next frontier without copying robots.
        current_passers = std::move(all_receivers);
    }

    return num_passes_to_robot;
}
}  // namespace

std::optional<std::pair<int, std::optional<Robot>>> getNumPassesToRobot(
    const Robot& initial_passer, const Robot& final_receiver, const Team& passing_team,
    const Team& other_team)
{
    if (initial_passer == final_receiver)
    {
        return std::make_pair(0, std::nullopt);
    }

    auto num_passes_to_robot =
        getNumPassesToAllRobots(initial_passer, passing_team, other_team);
    auto pass_data = num_passes_to_robot.find(final_receiver);
c
    // If we have checked all the robots we can and still haven't found the robot we
    // are looking for, it must be blocked and unable to be passed to in the current
    // state. Therefore, we return an std::nullopt
    if (pass_data == num_passes_to_robot.end())
    {
        return std::nullopt;
    }

    return pass_data->second;
}

void sortThreatsInDecreasingOrder(std::vector<EnemyThreat>& threats)
{
    // A lambda function that implements the '<' operator for the EnemyThreat struct
    // so it can be sorted. Lower threats are "less than" higher threats.
    auto enemyThreatLessThanComparator = [](const EnemyThreat& a, const EnemyThreat& b)
    {
        // Robots with the ball are more threatening than robots without the ball, and
        // robots with the ball are the most threatening since they can shoot or move
        // the ball towards our net
        if (a.has_ball && !b.has_ball)
        {
            return false;
        }
        else if (!a.has_ball && b.has_ball)
        {
            return true;
        }
        // If both robots have the ball, the robot with a worse shot on our net is less
        // threatening (although this case is unlikely to happen since usually only 1
        // robot can have the ball at a time)
        else if (a.has_ball && b.has_ball)
        {
            return a.best_shot_angle < b.best_shot_angle;
        }
        else
        {
            // If neither robot has the ball, the robot that takes longer to reach via
            // passing is less threatening
            if (a.num_passes_to_get_possession < b.num_passes_to_get_possession)
            {
                return false;
            }
            else if (a.num_passes_to_get_possession > b.num_passes_to_get_possession)
            {
                return true;
            }
            else
            {
                // Finally, if both robots can be reached in the same number of passes,
                // the robot with a smaller view of the net is considered less
                // threatening. The reason we use goal_angle here rather than the
                // best_shot_angle is that the goal_angle doesn't change if the robot is
                // blocked from shooting (e.g. by a defender). This makes the evaluation
                // more stable since the value won't change drastically as our robots
                // move into defensive positions and change the best_shot_angle. If we had
                // fewer robots than the enemy team and were using the best_shot_angle,
                // defenders could oscillate between enemies since when the defender
                // blocks one enemy, the unblocked one becomes more threatening and the
                // defender would then move there.
                if (a.goal_angle != b.goal_angle)
                {
                    return a.goal_angle < b.goal_angle;
                }
                // Fully deterministic order so equal threats can't swap between ticks
                return a.robot.id() > b.robot.id();
            }
        }
    };

    // Sort threats from highest threat to lowest threat
    // Use reverse iterators to sort the vector in descending order
    std::sort(threats.rbegin(), threats.rend(), enemyThreatLessThanComparator);
}

void stabilizePrimaryThreat(std::vector<EnemyThreat>& threats,
                            std::optional<RobotId> previous_primary, const Angle& margin)
{
    if (!previous_primary || threats.size() < 2 ||
        threats.front().robot.id() == *previous_primary)
    {
        return;
    }

    auto previous = std::find_if(threats.begin(), threats.end(),
                                 [&](const EnemyThreat& threat)
                                 { return threat.robot.id() == *previous_primary; });
    if (previous == threats.end())
    {
        return;
    }

    const auto& leader = threats.front();
    if (previous->has_ball == leader.has_ball &&
        previous->num_passes_to_get_possession == leader.num_passes_to_get_possession &&
        leader.goal_angle.minDiff(previous->goal_angle) <= margin)
    {
        std::rotate(threats.begin(), previous, previous + 1);
    }
}

void holdPrimaryThreat(std::vector<EnemyThreat>& threats,
                       std::optional<RobotId> previous_primary, PrimaryThreatHold& state,
                       unsigned int hold_updates)
{
    if (!previous_primary || threats.empty())
    {
        state.updates_held = hold_updates;
        return;
    }
    if (threats.front().robot.id() == *previous_primary)
    {
        state.updates_held = std::min(state.updates_held + 1, hold_updates);
        return;
    }

    auto previous = std::find_if(threats.begin(), threats.end(),
                                 [&](const EnemyThreat& threat)
                                 { return threat.robot.id() == *previous_primary; });
    const bool previous_contests_ball =
        previous != threats.end() &&
        (previous->has_ball || previous->num_passes_to_get_possession <= 1);
    const bool leader_takes_ball =
        previous != threats.end() && threats.front().has_ball && !previous->has_ball;
    if (hold_updates == 0 || !previous_contests_ball || leader_takes_ball ||
        state.updates_held >= hold_updates)
    {
        state.updates_held = 0;
        return;
    }
    state.updates_held++;
    std::rotate(threats.begin(), previous, previous + 1);
}

std::vector<EnemyThreat> getAllEnemyThreats(
    const Field& field, const Team& friendly_team, Team enemy_team, const Ball& ball,
    bool include_goalie, std::optional<RobotId> previous_baller,
    const Duration& baller_switch_margin, std::optional<RobotId>* baller_out)
{
    if (!include_goalie && enemy_team.getGoalieId())
    {
        enemy_team.removeRobotWithId(*enemy_team.getGoalieId());
    }

    std::vector<EnemyThreat> threats;
    threats.reserve(enemy_team.numRobots());

    auto robot_with_effective_possession =
        getRobotWithEffectiveBallPossession(enemy_team, ball, field, previous_baller,
                                            baller_switch_margin);
    if (baller_out)
    {
        *baller_out = robot_with_effective_possession
                          ? std::make_optional(robot_with_effective_possession->id())
                          : std::nullopt;
    }

    NumPassesToRobotMap num_passes_to_robot;
    if (robot_with_effective_possession)
    {
        num_passes_to_robot = getNumPassesToAllRobots(
            robot_with_effective_possession.value(), enemy_team, friendly_team);
    }

    for (const auto& robot : enemy_team.getAllRobots())
    {
        bool has_ball            = robot.isNearDribbler(ball.position());
        const bool shot_eligible = canShootOnFriendlyGoal(field, robot.position());

        // Get the angle from the robot to each friendly goalpost, then find the
        // difference between these angles to get the goal_angle for the robot.
        // Robots in or behind our goal mouth are close to both goalposts, so the angle
        // between them is wide even though they have no net to shoot at. Give them an
        // angle of zero
        Angle goal_angle = Angle::zero();
        if (shot_eligible)
        {
            auto friendly_goalpost_angle_1 =
                (field.friendlyGoalpostPos() - robot.position()).orientation();
            auto friendly_goalpost_angle_2 =
                (field.friendlyGoalpostNeg() - robot.position()).orientation();
            goal_angle = friendly_goalpost_angle_1.minDiff(friendly_goalpost_angle_2);
        }

        std::optional<Angle> best_shot_angle  = std::nullopt;
        std::optional<Point> best_shot_target = std::nullopt;
        auto best_shot_data =
            shot_eligible
                ? calcBestShotOnGoal(field, friendly_team, enemy_team, robot.position(),
                                     TeamType::FRIENDLY, {robot})
                : std::nullopt;
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
        auto pass_data              = num_passes_to_robot.find(robot);
        if (pass_data != num_passes_to_robot.end())
        {
            num_passes = pass_data->second.first;
            passer     = pass_data->second.second;
        }

        EnemyThreat threat{robot,           has_ball,         goal_angle,
                           best_shot_angle, best_shot_target, num_passes,
                           passer};

        threats.emplace_back(threat);
    }

    // Sort the threats so the "most threatening threat" is first in the vector, and the
    // "least threatening threat" is last in the vector
    sortThreatsInDecreasingOrder(threats);

    return threats;
}
