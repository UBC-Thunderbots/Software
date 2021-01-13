#include "software/ai/hl/stp/play/penalty_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/friendly_scored_validation.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class PenaltyKickPlayTest : public SimulatedTestFixture
{
};

TEST_F(PenaltyKickPlayTest, test_penalty_kick_setup)
{
    setBallState(BallState(field().penaltyEnemy(), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-2, -2), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(2, -3.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(
        TestUtil::createStationaryRobotStatesWithId({field().enemyGoalCenter()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickPlay));
    setRefereeCommand(RefereeCommand::PREPARE_PENALTY_US, RefereeCommand::NORMAL_START);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(9.5));
}

TEST_F(PenaltyKickPlayTest, test_penalty_kick_take)
{
    Vector behind_ball_direction =
        (field().penaltyEnemy() - field().enemyGoalpostPos()).normalize();

    Point behind_ball = field().penaltyEnemy() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS + 0.1);
    setBallState(BallState(field().penaltyEnemy(), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-4, 0), behind_ball, Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(2, -3.1)}));
    setFriendlyGoalie(0);
    addEnemyRobots(
        TestUtil::createStationaryRobotStatesWithId({field().enemyGoalCenter()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(PenaltyKickPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_PENALTY_US);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1396
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
