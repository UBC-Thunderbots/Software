#include "software/ai/hl/stp/play/corner_kick_play.h"

#include <gtest/gtest.h>

#include <utility>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class CornerKickPlayTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(CornerKickPlayTest, test_corner_kick_play_bottom_left)
{
    BallState ball_state(Point(4.5, -3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setFriendlyAIPlay(TYPENAME(CornerKickPlay));
    setEnemyAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotReceivedBall(5, world_ptr, yield);
            friendlyScored(world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(CornerKickPlayTest, test_corner_kick_play_top_right)
{
    BallState ball_state(Point(4.5, 3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(0, 1.5), Point(0, 0.5), Point(0, -0.5), Point(0, -1.5),
         Point(4.6, 3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setFriendlyAIPlay(TYPENAME(CornerKickPlay));
    setEnemyAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            robotReceivedBall(5, world_ptr, yield);
            friendlyScored(world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

class CornerKickPlayInvariantAndIsApplicableTest
    : public SimulatedPlayTestFixture,
      public ::testing::WithParamInterface<
          std::tuple<TeamSide, RefereeCommand, RefereeCommand, Point, bool, bool>>
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_P(CornerKickPlayInvariantAndIsApplicableTest, test_invariant_and_is_applicable)
{
    // Set world variables
    auto play_config = std::make_shared<ThunderbotsConfig>()->getPlayConfig();

    auto world = ::TestUtil::createBlankTestingWorld();

    auto corner_kick_play = CornerKickPlay(play_config);

    // -------------------------------------------------------------------------
    // Check if isApplicable() and invariantHolds() for various world states and ball
    // positions

    // initialize world state from parameters
    world.setTeamWithPossession(std::get<0>(GetParam()));
    world.updateGameState(
        ::TestUtil::createGameState(std::get<1>(GetParam()), std::get<2>(GetParam())));

    // initialize ball from parameters
    BallState ball_state_in_corner(std::get<3>(GetParam()), Vector(0, 0));
    Timestamp time_stamp;

    Ball ball(ball_state_in_corner, time_stamp);
    world.updateBall(ball);

    // assert that isApplicable() is true or false depending on testcase
    EXPECT_EQ(corner_kick_play.isApplicable(world), std::get<4>(GetParam()));

    // assert that invariantHolds() is true or false depending on testcase
    EXPECT_EQ(corner_kick_play.invariantHolds(world), std::get<5>(GetParam()));
}

INSTANTIATE_TEST_CASE_P(
    BallPositions, CornerKickPlayInvariantAndIsApplicableTest,
    ::testing::Values(
        // Test that both invariantHolds() and isApplicable() for ball in the enemy left
        // corner with variable position
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, -3), true, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.3, -3), true, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, -2.8), true, true),

        // Test that both invariantHolds() and isApplicable() for ball in the enemy right
        // corner with variable position
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, 3), true, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.3, 3), true, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, 2.8), true, true),

        // Test that invariantHolds() and !isApplicable() for ball not in a corner
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(0, 0), false, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(-4.5, 3), false, true),
        std::make_tuple(TeamSide::FRIENDLY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(-4.5, -3), false, true),

        // Test that !invariantHolds() and isApplicable() for ball in corner but not our
        // possession
        std::make_tuple(TeamSide::ENEMY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, -3), true, false),
        std::make_tuple(TeamSide::ENEMY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(4.5, 3), true, false),

        // Test that !invariantHolds() and !isApplicable() for ball not corner and not our
        // possession
        std::make_tuple(TeamSide::ENEMY, RefereeCommand::DIRECT_FREE_US,
                        RefereeCommand::HALT, Point(0, 0), false, false)));
