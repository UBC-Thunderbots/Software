#include "software/ai/hl/stp/play/halt_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/game_state.h"
#include "software/world/world.h"

class HaltPlayTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(HaltPlayTest, test_halt_play)
{
    BallState ball_state(Point(0, 0.5), Vector(0, 0));
    auto friendly_robots = TestUtil::createMovingRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(4.6, -3.1)},
        {Vector(1, 1), Vector(2, 1), Vector(1, 2), Vector(2, 2), Vector(1, 1),
         Vector(2, 1)});
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);

    std::vector<ValidationFunction> terminating_validation_functions = {

        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            for (unsigned i = 0; i < 1000; i++)
            {
                robotHalt(world_ptr, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}

// HaltPlayInvariantAndIsApplicableTest is the test suite, a collection of test cases.
// test_invariant_and_is_applicable is the name of the first test case in the suite.
//
// If there were more test cases, we would duplicate the following TEST(...) {}
// block and give the next test case a descriptive name.
TEST(HaltPlayInvariantAndIsApplicableTest, test_invariant_and_is_applicable)
{
    // Lets setup some things we need to run this test:
    //
    // Dynamic Parameter Config: This data structure is passed into the play and contains
    // runtime configurable values.  We don't need to change anything here we just need to
    // pass it in.
    auto play_config = std::make_shared<ThunderbotsConfig>()->getPlayConfig();

    // World: A blank testing world we will manipulate for the test
    auto world = ::TestUtil::createBlankTestingWorld();

    // HaltPlay: The play under test
    auto halt_play = HaltPlay(play_config);

    // GameState: The game state to test with. For this test we don't care about
    // RestartReason and our_restart. Looking at software/world/game_state.cpp, we
    // only need to init the play state to HALT
    world.updateGameState(
        ::TestUtil::createGameState(RefereeCommand::HALT, RefereeCommand::HALT));

    // Lets make sure the play will start running and stay running.
    ASSERT_TRUE(halt_play.isApplicable(world));
    ASSERT_TRUE(halt_play.invariantHolds(world));

    // Now lets make sure that we don't run when are NOT halted
    world.updateGameState(
        ::TestUtil::createGameState(RefereeCommand::NORMAL_START, RefereeCommand::HALT));

    // Make sure we don't run the halt play
    ASSERT_FALSE(halt_play.isApplicable(world));
    ASSERT_FALSE(halt_play.invariantHolds(world));
}
