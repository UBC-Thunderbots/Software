#include "software/ai/evaluation/defender_assignment.h"

#include <gtest/gtest.h>

#include <algorithm>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/test_util/test_util.h"

class DefenderAssignmentTest : public ::testing::Test
{
   protected:
    TbotsProto::DefensePlayConfig::DefenderAssignmentConfig config;
};
class GetAllDefenderAssignmentsTest : public DefenderAssignmentTest
{
};
class FilterOutSimilarThreatsTest : public DefenderAssignmentTest
{
};
class GroupGoalLanesByDensityTest : public DefenderAssignmentTest
{
};

TEST_F(GetAllDefenderAssignmentsTest, no_threats)
{
    auto world   = TestUtil::createBlankTestingWorld();
    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);

    auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);

    // Make sure we got the correct number of assignments
    EXPECT_EQ(assignments.size(), 0);
}

TEST_F(GetAllDefenderAssignmentsTest, single_threat)
{
    Point threat_position(1, 1);
    auto world      = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), {threat_position}, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(enemy_team);

    Point ball_position(-1, -1);
    world->updateBall(Ball(ball_position, Vector(), Timestamp()));

    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);

    auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);

    // Make sure we got the correct number of assignments
    EXPECT_EQ(assignments.size(), 1);

    // The primary threat is defended from the ball's position
    auto assignment = assignments[0];
    EXPECT_EQ(assignment.type, DefenderAssignmentType::CREASE_DEFENDER);
    EXPECT_EQ(assignment.target, ball_position);
}

TEST_F(GetAllDefenderAssignmentsTest,
       primary_threat_is_defended_from_the_ball_and_others_from_their_own_position)
{
    auto world = TestUtil::createBlankTestingWorld();

    Robot first_threat  = Robot(0, Point(1, 1), Vector(), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot second_threat = Robot(1, Point(2, 0), Vector(), Angle::zero(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<EnemyThreat> threats = {
        EnemyThreat{first_threat, false, Angle::fromDegrees(30), Angle::fromDegrees(30),
                    Point(-4.5, 0), 0, std::nullopt},
        EnemyThreat{second_threat, false, Angle::fromDegrees(20), Angle::fromDegrees(20),
                    Point(-4.5, 0), 1, first_threat},
    };

    auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);

    std::vector<Point> crease_targets;
    for (const auto& assignment : assignments)
    {
        if (assignment.type == DefenderAssignmentType::CREASE_DEFENDER)
        {
            crease_targets.emplace_back(assignment.target);
        }
    }

    // The primary threat's goal lane starts at the ball, every other threat's goal lane
    // starts at its own position
    EXPECT_NE(std::find(crease_targets.begin(), crease_targets.end(),
                        world->ball().position()),
              crease_targets.end());
    EXPECT_EQ(
        std::find(crease_targets.begin(), crease_targets.end(), first_threat.position()),
        crease_targets.end());
    EXPECT_NE(
        std::find(crease_targets.begin(), crease_targets.end(), second_threat.position()),
        crease_targets.end());
}

TEST_F(FilterOutSimilarThreatsTest, no_similar_threats)
{
    // Threats are spread out and should all be considered non-similar
    // to one another
    std::vector<Point> threat_positions = {Point(-2, -2), Point(-1, -2), Point(-2, 2),
                                           Point(-1, 2)};

    auto world      = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(enemy_team);

    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);

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
    std::vector<Point> threat_positions = {Point(0, 0), Point(0.15, 0.15),
                                           Point(-0.15, -0.15)};

    auto world      = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(enemy_team);

    // Place ball in front of robot positioned at (0, 0) so that it becomes
    // the primary threat
    world->updateBall(Ball(Point(0.1, 0), Vector(), Timestamp()));

    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);

    auto filtered_threats = filterOutSimilarThreats(
        threats, config.min_distance_between_threats_meters(),
        Angle::fromDegrees(config.min_angle_between_threats_degrees()));

    // Only primary threat should remain in filtered list
    std::vector<EnemyThreat> expected_result = {threats[0]};
    EXPECT_EQ(filtered_threats, expected_result);
}

TEST_F(FilterOutSimilarThreatsTest, filter_out_similarly_angled_threats)
{
    // Secondary threats have same angle to the primary threat at (0, 0)
    // and should be considered similar
    std::vector<Point> threat_positions = {Point(0, 0), Point(2, 2), Point(4, 4)};

    auto world      = TestUtil::createBlankTestingWorld();
    auto enemy_team = TestUtil::setRobotPositionsHelper(
        world->enemyTeam(), threat_positions, Timestamp::fromSeconds(0));
    world->updateEnemyTeamState(enemy_team);

    // Place ball in front of robot positioned at (0, 0) so that it becomes
    // the primary threat
    world->updateBall(Ball(Point(0.1, 0), Vector(), Timestamp()));

    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);

    auto filtered_threats = filterOutSimilarThreats(
        threats, config.min_distance_between_threats_meters(),
        Angle::fromDegrees(config.min_angle_between_threats_degrees()));

    // Only threat furthest away from the primary threat should be
    // filtered out
    std::vector<EnemyThreat> expected_result = {threats[0], threats[1]};
    EXPECT_EQ(filtered_threats, expected_result);
}

TEST_F(GroupGoalLanesByDensityTest, single_dense_cluster)
{
    Field field = Field::createField(TbotsProto::FieldType::DIV_B);

    std::vector<Point> threat_positions = {Point(-1, -1), Point(-1, -1.5), Point(-1, 0),
                                           Point(-1, 2)};

    // Create goal lanes from threat positions
    std::vector<GoalLane> goal_lanes;
    for (const auto& threat_position : threat_positions)
    {
        auto lane          = Segment(threat_position, field.friendlyGoalCenter());
        auto angle_to_goal = lane.reverse().toVector().orientation();
        // Threat rating doesn't matter for this test, so it's set to 0
        goal_lanes.emplace_back(GoalLane{{lane, 0}, angle_to_goal});
    }

    auto grouped_goal_lanes =
        groupGoalLanesByDensity(goal_lanes, config.goal_lane_density_threshold());

    // Goal lanes created from first two threats should be close in angle to goal
    // since the threats are positioned near each other, so they should be grouped
    // together as a dense cluster
    std::vector<std::vector<GoalLane>> expected_result = {
        {goal_lanes[0], goal_lanes[1]},
        {goal_lanes[2]},
        {goal_lanes[3]},
    };

    EXPECT_EQ(grouped_goal_lanes, expected_result);
}

TEST_F(GroupGoalLanesByDensityTest, multiple_dense_clusters)
{
    Field field = Field::createField(TbotsProto::FieldType::DIV_B);

    std::vector<Point> threat_positions = {Point(-1, -1), Point(-1, -1.5), Point(-1, 0),
                                           Point(-1, 2.5), Point(-1, 2)};

    // Create goal lanes from threat positions
    std::vector<GoalLane> goal_lanes;
    for (const auto& threat_position : threat_positions)
    {
        auto lane          = Segment(threat_position, field.friendlyGoalCenter());
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
    std::vector<std::vector<GoalLane>> expected_result = {
        {goal_lanes[0], goal_lanes[1]},
        {goal_lanes[2]},
        {goal_lanes[3], goal_lanes[4]},
    };

    EXPECT_EQ(grouped_goal_lanes, expected_result);
}

TEST_F(GetAllDefenderAssignmentsTest, ineligible_shooters_keep_pass_coverage)
{
    auto world     = TestUtil::createBlankTestingWorld();
    Point excluded = world->field().friendlyGoalCenter();
    TestUtil::setEnemyRobotPositions(world, {Point(0, 2), excluded}, Timestamp());
    TestUtil::setBallPosition(world, Point(0.09, 2), Timestamp());
    auto threats     = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                          world->enemyTeam(), world->ball(), false);
    auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);
    bool pass_found = false;
    for (const auto& assignment : assignments)
    {
        if (assignment.type == CREASE_DEFENDER)
            EXPECT_NE(assignment.target, excluded);
        else
            pass_found = true;
    }
    EXPECT_TRUE(pass_found);
    auto excluded_threat = std::find_if(threats.begin(), threats.end(),
                                        [&](const auto& threat)
                                        { return threat.robot.position() == excluded; });
    ASSERT_NE(excluded_threat, threats.end());
    EXPECT_EQ(excluded_threat->num_passes_to_get_possession, 1);
    EXPECT_TRUE(excluded_threat->passer);
}

TEST_F(GetAllDefenderAssignmentsTest, eligible_blocked_shooters_keep_crease_coverage)
{
    auto world = TestUtil::createBlankTestingWorld();
    const Point shooter(-2, 0);
    TestUtil::setEnemyRobotPositions(world, {shooter}, Timestamp());
    TestUtil::setFriendlyRobotPositions(world, {Point(-2.2, 0)}, Timestamp());
    TestUtil::setBallPosition(world, shooter, Timestamp());
    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);
    ASSERT_EQ(threats.size(), 1);
    EXPECT_FALSE(threats[0].best_shot_target);
    auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);
    ASSERT_EQ(assignments.size(), 1);
    EXPECT_EQ(assignments[0].type, CREASE_DEFENDER);
    EXPECT_EQ(assignments[0].target, shooter);
}

TEST_F(GetAllDefenderAssignmentsTest, crease_coverage_obeys_shot_eligibility_boundaries)
{
    for (double offset :
         {-0.1, 0.0, ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS + 1e-9})
    {
        auto world    = TestUtil::createBlankTestingWorld();
        Point shooter = world->field().friendlyGoalCenter() + Vector(offset, 0);
        TestUtil::setEnemyRobotPositions(world, {shooter}, Timestamp());
        TestUtil::setBallPosition(world, shooter, Timestamp());
        auto threats     = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                              world->enemyTeam(), world->ball(), false);
        auto assignments =
            getAllDefenderAssignments(threats, world->field(), world->ball(), config);
        EXPECT_EQ(assignments.size(), offset > ROBOT_MAX_RADIUS_METERS ? 1 : 0);
    }
}

TEST_F(FilterOutSimilarThreatsTest, empty_input_is_safe)
{
    EXPECT_TRUE(filterOutSimilarThreats({}, 0, Angle::zero()).empty());
}

TEST_F(GetAllDefenderAssignmentsTest, zero_distance_threshold_does_not_create_self_pass)
{
    auto world = TestUtil::createBlankTestingWorld();
    TestUtil::setEnemyRobotPositions(world, {Point(0, 1)}, Timestamp());
    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);
    ASSERT_EQ(threats.size(), 1);
    config.set_min_distance_between_threats_meters(0);
    config.set_min_angle_between_threats_degrees(0);
    const auto assignments =
        getAllDefenderAssignments(threats, world->field(), world->ball(), config);
    ASSERT_EQ(assignments.size(), 1);
    EXPECT_EQ(assignments.front().type, CREASE_DEFENDER);
}

TEST_F(FilterOutSimilarThreatsTest, coincident_threats_do_not_create_zero_length_lanes)
{
    auto world = TestUtil::createBlankTestingWorld();
    TestUtil::setEnemyRobotPositions(world, {Point(0, 1), Point(0, 1), Point(1, 1)},
                                     Timestamp());
    auto threats = getAllEnemyThreats(world->field(), world->friendlyTeam(),
                                      world->enemyTeam(), world->ball(), false);
    // Explicitly choose the two overlapping robots as the primary and secondary.
    std::sort(threats.begin(), threats.end(),
              [](const auto& a, const auto& b) { return a.robot.id() < b.robot.id(); });
    const auto filtered = filterOutSimilarThreats(threats, 0, Angle::zero());
    ASSERT_EQ(filtered.size(), 2);
    EXPECT_EQ(filtered[0].robot.id(), 0);
    EXPECT_EQ(filtered[1].robot.id(), 2);
}

class AssignmentHysteresisTest : public DefenderAssignmentTest
{
};

TEST_F(AssignmentHysteresisTest, previously_chosen_assignment_keeps_its_spot)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
        {CREASE_DEFENDER, Point(-1, -1), 8.0},
    };
    // The slightly worse assignment was chosen last update, so it should stay on top
    std::vector<DefenderAssignment> previous = {{CREASE_DEFENDER, Point(-1, -1.1), 6.0}};

    applyAssignmentHysteresis(assignments, previous, 1.5, 0.5);

    ASSERT_EQ(assignments.size(), 2);
    EXPECT_EQ(assignments[0].target, Point(-1, -1));
    EXPECT_DOUBLE_EQ(assignments[0].coverage_rating, 12.0);
    EXPECT_EQ(assignments[1].target, Point(-1, 1));
    EXPECT_DOUBLE_EQ(assignments[1].coverage_rating, 9.0);
}

TEST_F(AssignmentHysteresisTest, a_much_better_assignment_still_wins)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 30.0},
        {CREASE_DEFENDER, Point(-1, -1), 8.0},
    };
    std::vector<DefenderAssignment> previous = {{CREASE_DEFENDER, Point(-1, -1), 8.0}};

    applyAssignmentHysteresis(assignments, previous, 1.5, 0.5);

    EXPECT_EQ(assignments[0].target, Point(-1, 1));
}

TEST_F(AssignmentHysteresisTest, multiplier_of_one_changes_nothing)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
        {CREASE_DEFENDER, Point(-1, -1), 8.0},
    };
    const auto original = assignments;
    std::vector<DefenderAssignment> previous = {{CREASE_DEFENDER, Point(-1, -1), 8.0}};

    applyAssignmentHysteresis(assignments, previous, 1.0, 0.5);

    EXPECT_EQ(assignments, original);
}

TEST_F(AssignmentHysteresisTest, no_previous_assignments_changes_nothing)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
        {PASS_DEFENDER, Point(-1, -1), 8.0},
    };
    const auto original = assignments;

    applyAssignmentHysteresis(assignments, {}, 1.5, 0.5);

    EXPECT_EQ(assignments, original);
}

TEST_F(AssignmentHysteresisTest, different_type_or_distant_target_is_not_a_match)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
        {PASS_DEFENDER, Point(-1, -1), 8.0},
    };
    const auto original = assignments;
    std::vector<DefenderAssignment> previous = {
        // Same place as the pass assignment, but a crease defender
        {CREASE_DEFENDER, Point(-1, -1), 8.0},
        // Same type as the crease assignment, but too far away to be the same one
        {CREASE_DEFENDER, Point(-1, 1.6), 9.0},
    };

    applyAssignmentHysteresis(assignments, previous, 1.5, 0.5);

    EXPECT_EQ(assignments, original);
}

TEST_F(AssignmentHysteresisTest, double_crease_coverage_boosts_the_assignment_once)
{
    std::vector<DefenderAssignment> assignments = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
    };
    // Two crease defenders were assigned to the same target last update
    std::vector<DefenderAssignment> previous = {
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
        {CREASE_DEFENDER, Point(-1, 1), 9.0},
    };

    applyAssignmentHysteresis(assignments, previous, 1.5, 0.5);

    EXPECT_DOUBLE_EQ(assignments[0].coverage_rating, 13.5);
}

TEST_F(AssignmentHysteresisTest, equal_ratings_keep_their_relative_order)
{
    std::vector<DefenderAssignment> assignments = {
        {PASS_DEFENDER, Point(-1, 1), 5.0},
        {PASS_DEFENDER, Point(-1, 2), 5.0},
        {PASS_DEFENDER, Point(-1, 3), 5.0},
    };
    std::vector<DefenderAssignment> previous = {{CREASE_DEFENDER, Point(5, 5), 1.0}};
    const auto original = assignments;

    applyAssignmentHysteresis(assignments, previous, 1.5, 0.5);

    EXPECT_EQ(assignments, original);
}
