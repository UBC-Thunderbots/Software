#include "software/ai/hl/stp/play/corner_kick_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class CornerKickPlayTest : public SimulatedTestFixture
{
};

TEST_F(CornerKickPlayTest, test_corner_kick_play)
{
    setBallState(BallState(Point(4.5, -3), Vector(0, 0)));
    addFriendlyRobots({
        RobotStateWithId{
            .id          = 0,
            .robot_state = RobotState(Point(-3, 2.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 1,
            .robot_state = RobotState(Point(-3, 1.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 2,
            .robot_state = RobotState(Point(-3, 0.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 3,
            .robot_state = RobotState(Point(-3, -0.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 4,
            .robot_state = RobotState(Point(-3, -1.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 5,
            .robot_state = RobotState(Point(4.6, -3.1), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
    });
    setFriendlyGoalie(0);
    addEnemyRobots({
        RobotStateWithId{
            .id          = 0,
            .robot_state = RobotState(Point(1, 0), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 1,
            .robot_state = RobotState(Point(1, 2.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 2,
            .robot_state = RobotState(Point(1, -2.5), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())},
        RobotStateWithId{
            .id          = 3,
            .robot_state = RobotState(field().enemyGoalCenter(), Vector(0, 0),
                                      Angle::zero(), AngularVelocity::zero())},
        RobotStateWithId{.id          = 4,
                         .robot_state = RobotState(
                             field().enemyDefenseArea().negXNegYCorner(), Vector(0, 0),
                             Angle::zero(), AngularVelocity::zero())},
        RobotStateWithId{.id          = 5,
                         .robot_state = RobotState(
                             field().enemyDefenseArea().negXPosYCorner(), Vector(0, 0),
                             Angle::zero(), AngularVelocity::zero())},
    });

    setEnemyGoalie(0);
    setAIPlay(CornerKickPlay::name);
    setRefboxGameState(RefboxGameState::NORMAL_START, RefboxGameState::INDIRECT_FREE_US);

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
