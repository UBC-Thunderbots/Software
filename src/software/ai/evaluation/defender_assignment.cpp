#include "software/ai/evaluation/defender_assignment.h"

#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"
#include "software/math/math_functions.h"

std::vector<DefenderAssignment> getAllDefenderAssignments(
    const std::vector<EnemyThreat> &threats, const Field &field, const Ball &ball)
{
    std::vector<ShootingLane> goal_lanes;
    std::vector<ShootingLane> passing_lanes;
    std::vector<DefenderAssignment> assignments;

    // Get filtered list of threats with similarly positioned threats removed
    auto filtered_threats = filterOutSimilarThreats(threats);

    // Construct passing lanes and determine pass defender assignments
    auto primary_threat_position = filtered_threats.front().robot.position();
    for (unsigned int i = 1; i < filtered_threats.size(); i++)
    {
        auto lane =
            Segment(primary_threat_position, filtered_threats.at(i).robot.position());
        double threat_rating = static_cast<double>(filtered_threats.size()) - i;
        passing_lanes.emplace_back(ShootingLane{lane, threat_rating});
        assignments.emplace_back(
            DefenderAssignment{PASS_DEFENDER, lane.midPoint(), threat_rating});
    }

    // Construct goal lanes.
    // Using full list of threats (not filtered threats) since we need to
    // find potential goal lane from every enemy on the field
    for (unsigned int i = 0; i < threats.size(); i++)
    {
        auto threat_position =
            (i == 0) ? ball.position() : threats.at(i).robot.position();
        auto lane = Segment(threat_position, field.friendlyGoalCenter());
        double threat_rating =
            (static_cast<double>(threats.size()) - i) * GOAL_LANE_THREAT_MULTIPLIER;
        goal_lanes.emplace_back(ShootingLane{lane, threat_rating});
    }

    auto grouped_goal_lanes = groupGoalLanesByDensity(goal_lanes);

    // Determine crease defender assignments. 
    for (const auto &goal_lanes_group : grouped_goal_lanes)
    {
        double density_bonus = (goal_lanes_group.size() == 1) ? 0.5 : 0;
        for (const auto &goal_lane : goal_lanes_group)
        {
            auto threat_position = goal_lane.lane.getStart();
            double coverage_rating = goal_lane.threat_rating + density_bonus;

            // We let the target of the defender assignment be the location
            // of the originating enemy threat to cooperate with control
            // params for CreaseDefenderTactic
            assignments.emplace_back(
                DefenderAssignment{CREASE_DEFENDER, threat_position, coverage_rating});
        }
    }

    // Remove assignments with targets in the defense area
    assignments.erase(std::remove_if(assignments.begin(), assignments.end(),
        [&field](const auto &assignment) {
            return field.pointInFriendlyDefenseArea(assignment.target);
        }),
        assignments.end());

    // Sort the potential assignments by coverage rating in descending order
    std::sort(assignments.begin(), assignments.end(), [](const auto &a, const auto &b) {
        return a.coverage_rating > b.coverage_rating;
    });

    return assignments;
}

std::vector<EnemyThreat> filterOutSimilarThreats(const std::vector<EnemyThreat> &threats)
{
    std::vector<EnemyThreat> filtered_threats;

    // The primary threat is always included in our list of filtered threats
    auto primary_threat = threats.front();
    filtered_threats.emplace_back(primary_threat);

    for (const auto &threat : threats)
    {
        double distance_between_threats =
            distance(threat.robot.position(), primary_threat.robot.position());

        if (distance_between_threats < MIN_DISTANCE_BETWEEN_THREATS)
        {
            continue;
        }

        bool threat_with_similar_angle = false;
        for (unsigned int i = 1; i < filtered_threats.size(); i++)
        {
            auto filtered_threat = filtered_threats[i];

            auto angle =
                convexAngle(filtered_threat.robot.position(),
                            primary_threat.robot.position(), threat.robot.position());

            if (angle < MIN_ANGLE_BETWEEN_THREATS)
            {
                // Only the closest threat to the primary threat is included in our
                // list of filtered threats
                if (distance(primary_threat.robot.position(), threat.robot.position()) <
                    distance(primary_threat.robot.position(),
                             filtered_threat.robot.position()))
                {
                    filtered_threats[i] = threat;
                }
                threat_with_similar_angle = true;
                break;
            }
        }

        if (!threat_with_similar_angle)
        {
            filtered_threats.emplace_back(threat);
        }
    }

    return filtered_threats;
}

std::vector<std::vector<ShootingLane>> groupGoalLanesByDensity(const std::vector<ShootingLane> &goal_lanes)
{
    std::vector<std::pair<ShootingLane, double>> lane_angle_pairs;
    for (const auto &goal_lane : goal_lanes)
    {
        auto angle_to_goal = goal_lane.lane.toVector().orientation();

        std::pair<ShootingLane, double> lane_angle_pair;
        lane_angle_pair = std::make_pair(goal_lane, angle_to_goal.toRadians());

        lane_angle_pairs.emplace_back(lane_angle_pair);
    }

    // Sort the pairs by angle to goal in increasing order
    std::sort(lane_angle_pairs.begin(), lane_angle_pairs.end(), 
        [](const auto &a, const auto &b) { return a.second < b.second; });

    std::vector<std::vector<std::pair<ShootingLane, double>>> groups;
    groups.emplace_back(std::vector<std::pair<ShootingLane, double>> {lane_angle_pairs[0]});

    for (unsigned int i = 1; i < lane_angle_pairs.size(); i++)
    {
        // If the percent diff between current item and the previous is more than threshold
        if (percent_difference(lane_angle_pairs[i].second, groups.back().back().second) > 0.4)
        {
            // Start a new group
            groups.emplace_back(std::vector<std::pair<ShootingLane, double>> {});
        }

        // Add the current item to the last group in groups
        groups.back().emplace_back(lane_angle_pairs[i]);
    }

    std::vector<std::vector<ShootingLane>> goal_lane_groups;
    for (const auto &group : groups)
    {
        std::vector<ShootingLane> goal_lane_group;
        for (const auto &pair : group)
        {
            goal_lane_group.emplace_back(pair.first);    
        }
        goal_lane_groups.emplace_back(goal_lane_group);
    }

    return goal_lane_groups;
}
