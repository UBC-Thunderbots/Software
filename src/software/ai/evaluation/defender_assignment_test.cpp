#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/defender_assignment.h"
#include "software/test_util/test_util.h"

class DefenderAssignmentTest : public ::testing::Test
{
   protected:
    TbotsProto::DefensePlayConfig::DefenderAssignmentConfig config;
};
class GetAllDefenderAssignmentsTest : public DefenderAssignmentTest {};
class FilterOutSimilarThreatsTest : public DefenderAssignmentTest {};
class GroupGoalLanesByDensityTest : public DefenderAssignmentTest {};

TEST_F(GetAllDefenderAssignmentsTest, get_defender_assignments)
{

}

TEST_F(FilterOutSimilarThreatsTest, no_similar_threats)
{
    // Threats are spread out and should all be considered non-similar 
    // to one another
    std::vector<Point> threat_positions =
        { Point(-2, -2), Point(-1, -2), Point(-2, 2), Point(-1, 2) };

    auto world = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world.updateEnemyTeamState(enemy_team);

    auto threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                      world.enemyTeam(), world.ball(), false);

    auto filtered_threats = filterOutSimilarThreats(
        threats, config.min_distance_between_threats_meters(),
        Angle::fromDegrees(config.min_angle_between_threats_degrees()));

    // No threats should be filtered out, so the expected result should be
    // equal to the original list of threats that was passed in
    EXPECT_EQ(filtered_threats, threats);
}

TEST_F(FilterOutSimilarThreatsTest, filter_out_closely_positioned_threats)
{
    // Threats are close together and should be considered similar
    std::vector<Point> threat_positions =
        { Point(0, 0), Point(0.15, 0.15), Point(-0.15, -0.15)};

    auto world = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world.updateEnemyTeamState(enemy_team);

    // Place ball in front of robot positioned at (0, 0) so that it becomes
    // the primary threat
    world.updateBall(Ball(Point(0.1, 0), Vector(), Timestamp()));

    auto threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                      world.enemyTeam(), world.ball(), false);

    auto filtered_threats = filterOutSimilarThreats(
        threats, config.min_distance_between_threats_meters(),
        Angle::fromDegrees(config.min_angle_between_threats_degrees()));

    // Only primary threat should remain in filtered list
    std::vector<EnemyThreat> expected_result = { threats[0] };
    EXPECT_EQ(filtered_threats, expected_result);
}

TEST_F(FilterOutSimilarThreatsTest, filter_out_similarly_angled_threats)
{
    // Secondary threats have same angle to the primary threat at (0, 0)
    // and should be considered similar
    std::vector<Point> threat_positions =
        { Point(0, 0), Point(2, 2), Point(4, 4)};

    auto world = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world.enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world.updateEnemyTeamState(enemy_team);

    // Place ball in front of robot positioned at (0, 0) so that it becomes
    // the primary threat
    world.updateBall(Ball(Point(0.1, 0), Vector(), Timestamp()));

    auto threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                      world.enemyTeam(), world.ball(), false);

    auto filtered_threats = filterOutSimilarThreats(
        threats, config.min_distance_between_threats_meters(),
        Angle::fromDegrees(config.min_angle_between_threats_degrees()));

    // Only threat furthest away from the primary threat should be
    // filtered out
    std::vector<EnemyThreat> expected_result = { threats[0], threats[1] };
    EXPECT_EQ(filtered_threats, expected_result);
}

TEST_F(GroupGoalLanesByDensityTest, single_dense_cluster)
{
    std::vector<Point> threat_positions = 
        { Point(-1, -1), Point(-1, -1.5), Point(-1, 0), Point(-1, 2) };

    // Create goal lanes from threat positions
    std::vector<GoalLane> goal_lanes;
    const Point FRIENDLY_GOAL_CENTER(-4.5, 0);
    for (const auto &threat_position : threat_positions)
    {
        auto lane            = Segment(threat_position, FRIENDLY_GOAL_CENTER);
        auto angle_to_goal = lane.reverse().toVector().orientation();
        // Threat rating doesn't matter for this test, so it's set to 0
        goal_lanes.emplace_back(GoalLane{{lane, 0}, angle_to_goal});
    }

    auto grouped_goal_lanes =
        groupGoalLanesByDensity(goal_lanes, config.goal_lane_density_threshold());

    // Goal lanes created from first two threats should be close in angle to goal
    // since the threats are positioned near each other, so they should be grouped
    // together as a dense cluster 
    std::vector<std::vector<GoalLane>> expected_result = 
    {
        { goal_lanes[0], goal_lanes[1] },
        { goal_lanes[2] },
        { goal_lanes[3] },
    };

    EXPECT_EQ(grouped_goal_lanes, expected_result);
}

TEST_F(GroupGoalLanesByDensityTest, multiple_dense_clusters)
{
    std::vector<Point> threat_positions = 
        { Point(-1, -1), Point(-1, -1.5), Point(-1, 0), Point(-1, 2.5), Point(-1, 2) };

    // Create goal lanes from threat positions
    std::vector<GoalLane> goal_lanes;
    const Point FRIENDLY_GOAL_CENTER(-4.5, 0);
    for (const auto &threat_position : threat_positions)
    {
        auto lane            = Segment(threat_position, FRIENDLY_GOAL_CENTER);
        auto angle_to_goal = lane.reverse().toVector().orientation();
        // Threat rating doesn't matter for this test, so it's set to 0
        goal_lanes.emplace_back(GoalLane{{lane, 0}, angle_to_goal});
    }

    auto grouped_goal_lanes =
        groupGoalLanesByDensity(goal_lanes, config.goal_lane_density_threshold());

    // Goal lanes created from first two threats should be close in angle to goal
    // since the threats are positioned near each other, so they should be grouped
    // together as a dense cluster.
    // Ditto for the last two goal lanes created from the last two threats.
    std::vector<std::vector<GoalLane>> expected_result = 
    {
        { goal_lanes[0], goal_lanes[1] },
        { goal_lanes[2] },
        { goal_lanes[3], goal_lanes[4] },
    };

    EXPECT_EQ(grouped_goal_lanes, expected_result);
}
