#include "software/ai/hl/stp/play/free_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/game_state.h"
#include "software/world/world.h"

class FreeKickPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    FieldType field_type = FieldType::DIV_B;
    Field field          = Field::createField(field_type);
};

class FreeKickPlayIsApplicableInvariantHoldsTest
    : public SimulatedErForceSimPlayTestFixture
{
   protected:
    std::shared_ptr<const PlayConfig> play_config =
        std::make_shared<ThunderbotsConfig>()->getPlayConfig();

    World world = ::TestUtil::createBlankTestingWorld();

    // FreeKickPlay: the play under test
    FreeKickPlay free_kick_play = FreeKickPlay(play_config);
};

TEST_F(FreeKickPlayTest, test_free_kick_play_on_enemy_half)
{
    BallState ball_state(Point(1.5, -3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 0), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4, -2.5)});

    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});

    setEnemyGoalie(0);
    setAIPlay(TYPENAME(FreeKickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(FreeKickPlayTest, test_free_kick_play_on_friendly_half)
{
    BallState ball_state(Point(-1.5, -3), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-4.5, 0), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(FreeKickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

TEST_F(FreeKickPlayIsApplicableInvariantHoldsTest,
       test_set_ball_center_friendly_possession)
{
    // Test 1: GameState sets to isOurFreeKick and isPlaying.
    // min_dist_to_corner >= cornerKickPlay:BALL_IN_CORNER_RADIUS.
    // Set team with possession to FRIENDLY.
    world.updateGameState(::TestUtil::createGameState(RefereeCommand::DIRECT_FREE_US,
                                                      RefereeCommand::HALT));

    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    world.setTeamWithPossession(TeamSide::FRIENDLY);

    EXPECT_TRUE(free_kick_play.isApplicable(world));
    EXPECT_TRUE(free_kick_play.invariantHolds(world));

    world.updateGameState(::TestUtil::createGameState(RefereeCommand::INDIRECT_FREE_US,
                                                      RefereeCommand::HALT));

    EXPECT_TRUE(free_kick_play.isApplicable(world));
    EXPECT_TRUE(free_kick_play.invariantHolds(world));

    world.updateGameState(::TestUtil::createGameState(RefereeCommand::FORCE_START,
                                                      RefereeCommand::DIRECT_FREE_US));

    EXPECT_FALSE(free_kick_play.isApplicable(world));
    EXPECT_TRUE(free_kick_play.invariantHolds(world));
}

TEST_F(FreeKickPlayIsApplicableInvariantHoldsTest,
       test_set_ball_enemy_corners_friendly_possesion)
{
    // Test 2: GameState is set to isOurFreeKick and
    // min_dist_to_corner < cornerKickPlay:BALL_IN_CORNER_RADIUS.
    // Team with possession is FRIENDLY
    world.updateGameState(::TestUtil::createGameState(RefereeCommand::DIRECT_FREE_US,
                                                      RefereeCommand::HALT));

    world = ::TestUtil::setBallPosition(world, world.field().enemyCornerPos(),
                                        Timestamp::fromSeconds(0));

    world.setTeamWithPossession(TeamSide::FRIENDLY);

    EXPECT_FALSE(free_kick_play.isApplicable(world));
    EXPECT_TRUE(free_kick_play.invariantHolds(world));

    world = ::TestUtil::setBallPosition(world, world.field().enemyCornerNeg(),
                                        Timestamp::fromSeconds(0));

    EXPECT_FALSE(free_kick_play.isApplicable(world));
    EXPECT_TRUE(free_kick_play.invariantHolds(world));
}

TEST_F(FreeKickPlayIsApplicableInvariantHoldsTest, test_set_ball_center_enemy_posession)
{
    // Test 3: GameState sets to isTheirFreeKick and isPlaying.
    // min_dist_to_corner > cornerKickPlay:BALL_IN_CORNER_RADIUS.
    // Team with possession is ENEMY.
    world.updateGameState(::TestUtil::createGameState(RefereeCommand::DIRECT_FREE_THEM,
                                                      RefereeCommand::HALT));

    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    world.setTeamWithPossession(TeamSide::ENEMY);

    EXPECT_FALSE(free_kick_play.isApplicable(world));
    EXPECT_FALSE(free_kick_play.invariantHolds(world));

    world.updateGameState(::TestUtil::createGameState(RefereeCommand::FORCE_START,
                                                      RefereeCommand::DIRECT_FREE_THEM));

    EXPECT_FALSE(free_kick_play.isApplicable(world));
    EXPECT_FALSE(free_kick_play.invariantHolds(world));
}
