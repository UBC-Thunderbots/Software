#include "software/ai/hl/stp/play/scoring_with_static_defenders_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/friendly_scored_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class ScoringWithStaticDefendersPlayTest
    : public SimulatedPlayTestFixture,
      public ::testing::WithParamInterface<
          std::tuple<std::vector<RobotStateWithId>, Point>>
{
   protected:
    Field field = Field::createHardwareChallengeField();
};

TEST_F(ScoringWithStaticDefendersPlayTest,
       test_scoring_with_static_defenders_play_stopped)
{
    BallState ball_state(Point(-0.8, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(4, 0), Point(0.5, 0), Point(-3, 1)});
    setFriendlyGoalie(0);
    setAIPlay(TYPENAME(ScoringWithStaticDefendersPlay));

    setRefereeCommand(RefereeCommand::STOP, RefereeCommand::HALT);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // This will keep the test running for 9.5 seconds to give everything enough
        // time to settle into position and be observed with the Visualizer
        // TODO (#2106): Implement proper validation
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, {}, terminating_validation_functions,
            non_terminating_validation_functions, Duration::fromSeconds(10));
}

TEST_P(ScoringWithStaticDefendersPlayTest,
       test_scoring_with_static_defenders_play_freekick)
{
    auto enemy_robots = std::get<0>(GetParam());
    setAIPlay(TYPENAME(ScoringWithStaticDefendersPlay));
    BallState ball_state(std::get<1>(GetParam()), Vector(0, 0));

    std::vector<Point> friendly_robot_positions = {
        Point(0, ball_state.position().y()),
        Point(0, ball_state.position().y() + 4 * ROBOT_MAX_RADIUS_METERS),
        Point(0, ball_state.position().y() - 4 * ROBOT_MAX_RADIUS_METERS)};

    auto friendly_robots =
        TestUtil::createStationaryRobotStatesWithId(friendly_robot_positions);

    setRefereeCommand(RefereeCommand::DIRECT_FREE_US, RefereeCommand::HALT);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            friendlyScored(world_ptr, yield);
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(20));
}

INSTANTIATE_TEST_CASE_P(
    RobotAndBallPositions, ScoringWithStaticDefendersPlayTest,
    ::testing::Values(
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0), Point(2, -0.75), Point(2, -0.55)}),
                        Point(0.2, -1.8)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0.175), Point(1.5, 0), Point(2, -0.175)}),
                        Point(0.9, 0)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0), Point(2, 0.5), Point(1, 0), Point(2, -0.5)}),
                        Point(0.2, 0)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0), Point(1.95, 0), Point(1.95, -0.5),
                             Point(1.95, 0.5)}),
                        Point(1.35, 0)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0.2), Point(2.85, 1.1), Point(2, 0.5),
                             Point(2, 0), Point(1.45, 1.75)}),
                        Point(2.85, 1.75)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0), Point(2.4, 1.25), Point(2, 0.95),
                             Point(2, 0.36), Point(1.45, -1.21)}),
                        Point(0.95, -1.55)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.91, -0.3), Point(2.6, 1.2), Point(2, 0.3),
                             Point(1.7, -0.35), Point(2.3, -1.3)}),
                        Point(2.8, -1.8)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, 0.1), Point(2.25, 1.2), Point(2.05, 0.8),
                             Point(2.05, 0.4), Point(2.05, -0.4), Point(2.05, -0.8)}),
                        Point(0.2, -1.8)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, -0.3), Point(2.05, 1.05), Point(2.2, -1.2),
                             Point(1.55, -0.6), Point(1.35, 0.35), Point(0.8, 1)}),
                        Point(2.8, -1.75)),
        std::make_tuple(TestUtil::createStationaryRobotStatesWithId(
                            {Point(2.9, -0.15), Point(2.4, 1.25), Point(2, 0.45),
                             Point(1.85, -0.3), Point(2, -0.9), Point(1.3, 0)}),
                        Point(1.3, 0.65))));
