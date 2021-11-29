#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class ShootOrPassPlayTest : public SimulatedPlayTestFixture
{
   protected:
    Field field = Field::createSSLDivisionBField();
};

TEST_F(ShootOrPassPlayTest, test_shoot_or_pass_play)
{
    BallState ball_state(Point(-4.4, 2.9), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
        field.friendlyGoalCenter(),
        Point(-4.5, 3.0),
        Point(-2, 1.5),
        Point(-2, 0.5),
        Point(-2, -1.7),
        Point(-2, -1.5),
    });
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
         field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(ShootOrPassPlay));
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(24.5))
            {
                yield("Timestamp not at 24.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(25));
}

TEST_F(ShootOrPassPlayTest, test_shoot_or_pass_play_with_keep_away)
{
    BallState ball_state(Point(-1.8, 1.8), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId({
        field.friendlyGoalCenter(),
        Point(-4.5, 3.0),
        Point(-2, 1.7),
        Point(-2, 0.5),
        Point(-2, -1.7),
        Point(-2, -1.5),
    });
    setFriendlyGoalie(0);
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-1.5, 1.8), Point(-1.6, 1.95), Point(-1.6, 1.65), Point(-1.4, 1.65),
         field.enemyGoalCenter(), field.enemyDefenseArea().negXNegYCorner(),
         field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(ShootOrPassPlay));
    setRefereeCommand(RefereeCommand::FORCE_START, RefereeCommand::STOP);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1971
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(24.5))
            {
                yield("Timestamp not at 24.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(25));
}

TEST(ShootOrPassPlayInvariantAndIsApplicableTest, test_invariant_and_is_applicable)
{
    // Lets setup some things we need to run this test:
    //
    // Dynamic Parameter Config: This data structure is passed into the play and contains
    // runtime configurable values.  We don't need to change anything here we just need to
    // pass it in.
    auto play_config = std::make_shared<ThunderbotsConfig>()->getPlayConfig();

    // World: A blank testing world we will manipulate for the test
    auto world = ::TestUtil::createBlankTestingWorld();

    // ShootOrPassPlay: The play under test
    auto shoot_or_pass_play = ShootOrPassPlay(play_config);
    world.updateGameState(
        ::TestUtil::createGameState(RefereeCommand::FORCE_START, RefereeCommand::HALT));
    world.setTeamWithPossession(TeamSide::FRIENDLY);

    // Make sure play is running
    ASSERT_TRUE(shoot_or_pass_play.isApplicable(world));
    ASSERT_TRUE(shoot_or_pass_play.invariantHolds(world));

    world.setTeamWithPossession(TeamSide::ENEMY);

    // Make sure play is not running when enemy has the ball
    ASSERT_FALSE(shoot_or_pass_play.isApplicable(world));
    ASSERT_FALSE(shoot_or_pass_play.invariantHolds(world));

    // Set game state to a kickoff and start it. Game state is set to ready
    world.updateGameState(::TestUtil::createGameState(
        RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_US));
    world.setTeamWithPossession(TeamSide::FRIENDLY);

    // Move ball so that game state is set to playing
    world.updateGameStateBall(Ball(Point(0, 0), Vector(), Timestamp::fromSeconds(0)));
    world.updateGameStateBall(Ball(Point(0, 0.05), Vector(), Timestamp::fromSeconds(0)));

    // Make sure play is running
    ASSERT_TRUE(shoot_or_pass_play.isApplicable(world));
    ASSERT_TRUE(shoot_or_pass_play.invariantHolds(world));
}
