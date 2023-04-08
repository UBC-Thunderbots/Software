#include "software/ai/evaluation/defender_assignment.h"

#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"

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
        auto threat_rating = static_cast<unsigned int>(filtered_threats.size()) - i;
        passing_lanes.emplace_back(ShootingLane{lane, threat_rating});
        assignments.emplace_back(
            DefenderAssignment{PASS_DEFENDER, lane.midPoint(), threat_rating});
    }

    // Construct goal lanes and determine crease defender assignments.
    // Using full list of threats (not filtered threats) since we need to
    // find potential goal lane from every enemy on the field
    for (unsigned int i = 0; i < threats.size(); i++)
    {
        auto threat_position =
            (i == 0) ? ball.position() : threats.at(i).robot.position();
        auto lane = Segment(threat_position, field.friendlyGoalCenter());
        auto threat_rating =
            (static_cast<unsigned int>(threats.size()) - i) * GOAL_LANE_THREAT_MULTIPLIER;
        goal_lanes.emplace_back(ShootingLane{lane, threat_rating});

        // We let the target of the defender assignment be the location
        // of the originating enemy threat to cooperate with control
        // params for CreaseDefenderTactic
        assignments.emplace_back(
            DefenderAssignment{CREASE_DEFENDER, threat_position, threat_rating});
    }

    // Find points where goal and/or passing lanes intersect;
    // these are potential positions where we can place a pass defender
    // to block multiple lanes simultaneously
    for (auto const &goal_lane : goal_lanes)
    {
        for (auto const &passing_lane : passing_lanes)
        {
            auto intersections = intersection(goal_lane.lane, passing_lane.lane);
            if (intersections.size() == 1)
            {
                auto position = intersections.at(0);

                if (distance(position, goal_lane.lane.getStart()) < 0.5)
                {
                    continue;
                }

                auto coverage_rating = std::max(goal_lane.threat_rating, 
                                                passing_lane.threat_rating) + 1;

                assignments.emplace_back(DefenderAssignment{PASS_DEFENDER, position, coverage_rating});
            }
        }
    }

    // Remove assignments with targets in the defense area
    assignments.erase(
        std::remove_if(assignments.begin(), assignments.end(),
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
        auto distance_between_threats =
            distance(threat.robot.position(), primary_threat.robot.position());

        if (distance_between_threats < MIN_DISTANCE_BETWEEN_THREATS)
        {
            continue;
        }

        bool threat_with_similar_angle = false;
        for (unsigned int i = 1; i < filtered_threats.size(); i++)
        {
            auto grouped_threat = filtered_threats[i];

            auto angle =
                convexAngle(grouped_threat.robot.position(),
                            primary_threat.robot.position(), threat.robot.position());

            if (angle < MIN_ANGLE_BETWEEN_THREATS)
            {
                // Only the closest threat to the primary threat is included in our
                // list of filtered threats
                if (distance(primary_threat.robot.position(), threat.robot.position()) <
                    distance(primary_threat.robot.position(),
                             grouped_threat.robot.position()))
                {
                    filtered_threats[i] = grouped_threat;
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
