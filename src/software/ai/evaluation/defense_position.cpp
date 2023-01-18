#include "software/ai/evaluation/defense_position.h"

#include "software/geom/algorithms/intersection.h"

std::vector<DefensePosition> getAllDefensePositions(
    const std::vector<EnemyThreat> &threats, const Field &field)
{
    std::vector<DefenseLane> enemy_lanes;
    std::vector<DefensePosition> positions;

    // Construct passing lanes and determine pass defender positions
    auto primary_threat_position = threats.front().robot.position();
    for (unsigned int i = 1; i < threats.size(); i++)
    {
        auto lane = Segment(primary_threat_position, threats.at(i).robot.position());
        auto expected_threat = static_cast<unsigned int>(threats.size()) - i;
        enemy_lanes.emplace_back(DefenseLane{lane, expected_threat});
        positions.emplace_back(DefensePosition{lane.midPoint(), expected_threat, false});
    }

    // Multiplier to ensure that shooting lanes are scored higher than passing lanes
    static constexpr unsigned int SHOOTING_LANE_MULTIPLIER = 4;

    // Construct shooting lanes and determine potential crease defender positions
    for (unsigned int i = 0; i < threats.size(); i++)
    {
        auto lane = Segment(threats.at(i).robot.position(), field.friendlyGoalCenter());
        auto expected_threat =
            (static_cast<unsigned int>(threats.size()) - i) * SHOOTING_LANE_MULTIPLIER;
        enemy_lanes.emplace_back(DefenseLane{lane, expected_threat});
        positions.emplace_back(DefensePosition{lane.getStart(), expected_threat, true});
    }

    // Find points where shooting and/or passing lanes intersect --
    // these are potential positions where we can place a pass defender
    // to block multiple lanes simultaneously
    for (unsigned int i = 0; i < enemy_lanes.size(); i++)
    {
        for (unsigned int j = 0; j < enemy_lanes.size(); j++)
        {
            if (i == j)
            {
                continue;
            }

            auto intersections =
                intersection(enemy_lanes.at(i).lane, enemy_lanes.at(j).lane);
            if (intersections.size() == 1)
            {
                auto position = intersections.at(0);

                // Effectiveness of the position is combined expected threat of
                // both lanes used to create the position. This helps ensure
                // these positions blocking multiple lanes are scored as
                // "better" than normal pass defender positions blocking one
                // single lane
                auto effectiveness =
                    enemy_lanes.at(i).expected_threat + enemy_lanes.at(j).expected_threat;

                positions.emplace_back(DefensePosition{position, effectiveness, false});
            }
        }
    }

    // Sort the potential positions by effectiveness in descending order
    std::sort(positions.begin(), positions.end(),
              [](const auto &first, const auto &second) {
                  return first.effectiveness > second.effectiveness;
              });

    return positions;
}
