#include <gtest/gtest.h>

#include <cmath>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/game_state.h"
#include "software/world/world.h"

class BallOcclusionTest
    : public SimulatedErForceSimPlayTestFixture,
      public ::testing::WithParamInterface<std::tuple<
          BallState, std::vector<RobotStateWithId>, std::vector<RobotStateWithId>>>
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
};

TEST_P(BallOcclusionTest, test_ball_occlusion)
{
    BallState ball_state = std::get<0>(GetParam());
    auto friendly_robots = std::get<1>(GetParam());
    auto enemy_robots    = std::get<2>(GetParam());
    setFriendlyGoalie(0);
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(HaltPlay));
    setRefereeCommand(RefereeCommand::HALT, RefereeCommand::HALT);
    std::vector<ValidationFunction> terminating_validating_function = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
            {
                yield("Timestamp not at 9.5s");
            }
        }};
    std::vector<ValidationFunction> non_terminating_validating_function = {};
    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validating_function, non_terminating_validating_function,
            Duration::fromSeconds(10));
}

INSTANTIATE_TEST_CASE_P(
    BallOcclusionTests, BallOcclusionTest,
    ::testing::Values(
        std::make_tuple(BallState(Point(-4.5, 3), Vector(1, 0)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-4, 2.8), Point(-2.5, 2.8), Point(-1, 2.8),
                             Point(0.5, 2.8), Point(2, 2.8), Point(3.5, 2.8)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-4.5, -2), Vector(1, 0)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-4, -1.8), Point(-2.5, -1.8), Point(-1, -1.8),
                             Point(0.5, -1.8), Point(2, -1.8), Point(3.5, -1.8)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-4.5, 3), Vector(0, -1)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-4.3, -2), Point(-4.3, -1), Point(-4.3, 0),
                             Point(-4.3, 1), Point(-4.3, 2), Point(-4.3, 2.2)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(4.5, 3), Vector(0, -1)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(4.3, -2), Point(4.3, -1), Point(4.3, 0), Point(4.3, 1),
                             Point(4.3, 2), Point(4.3, 2.2)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)})),
        std::make_tuple(BallState(Point(-3, 0), Vector(-1, 0)),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(-2.8, 0)})),
        std::make_tuple(BallState(Point(3, 0), Vector(0, 1)),
                        TestUtil::createStationaryRobotStatesWithId({Point(2.8, 0)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-3.5, -1), Vector(-1, 1)),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)}),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-3.5 + 0.2 / sqrt(2.0), -1 - 0.2 / sqrt(2.0))})),
        std::make_tuple(BallState(Point(3.5, -1), Vector(1, 1)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(3.5 - 0.2 / sqrt(2.0), -1 - 0.2 / sqrt(2.0))}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-3.5, 1), Vector(-1, -1)),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)}),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-3.5 + 0.2 / sqrt(2.0), 1 + 0.2 / sqrt(2.0))})),
        std::make_tuple(BallState(Point(3.5, 1), Vector(1, -1)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(3.5 - 0.2 / sqrt(2), 1 + 0.2 / sqrt(2))}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-4.5 + 0.2 / sqrt(2.0), 3 - 0.2 / sqrt(2.0)),
                                  Vector(3.0 / sqrt(13.0), -2.0 / sqrt(13.0))),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 3),
                                                                     Point(-1.5, 1)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(-4.5 + 0.23 / sqrt(2.0), -3 + 0.23 / sqrt(2.0)),
                                  Vector(3.0 / sqrt(13.0), 2.0 / sqrt(13.0))),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, -3),
                                                                     Point(-1.5, -1)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(4.5 - 0.2 / sqrt(2.0), 3 - 0.2 / sqrt(2.0)),
                                  Vector(-3.0 / sqrt(13.0), -2.0 / sqrt(13.0))),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 3),
                                                                     Point(1.5, 1)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, 0)})),
        std::make_tuple(BallState(Point(4.5 - 0.23 / sqrt(2.0), -3 + 0.23 / sqrt(2.0)),
                                  Vector(-3.0 / sqrt(13.0), 2.0 / sqrt(13.0))),
                        TestUtil::createStationaryRobotStatesWithId({Point(4.5, -3),
                                                                     Point(1.5, -1)}),
                        TestUtil::createStationaryRobotStatesWithId({Point(-4.5, 0)})),
        std::make_tuple(BallState(Point(-4, -3), Vector(1, 3)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-3.9, -2.9), Point(-2.9, 0.1), Point(-1.9, 3.1)}),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-4.1, -3.1), Point(-3.1, -0.1), Point(-2.1, 2.9)})),
        std::make_tuple(BallState(Point(-4, -3), Vector(4, 1)),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-3.9, -2.9), Point(0.1, -1.9), Point(4.1, -0.9)}),
                        TestUtil::createStationaryRobotStatesWithId(
                            {Point(-4.1, -3.1), Point(-0.1, -2.1), Point(3.9, -1.1)}))));
