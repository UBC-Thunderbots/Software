#include "software/ai/hl/stp/tactic/stop/halt_tactic.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"

class HaltTacticTest : public SimulatedErForceSimPlayTestFixture
{
   protected:
    TbotsProto::FieldType field_type = TbotsProto::FieldType::DIV_B;
    Field field                      = Field::createField(field_type);
};

TEST_F(HaltTacticTest, robot_already_stopped)
{
    BallState ball_state(Point(0, 0.5), Vector(0, 0));

    auto friendly_robots = TestUtil::createMovingRobotStatesWithId(
        {Point(-3, 2.5), Point()}, {Vector(), Vector()});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<HaltTactic>();
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            for (unsigned i = 0; i < 1000; i++)
            {
                robotHalt(world_ptr, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(5));
}

TEST_F(HaltTacticTest, robot_start_moving)
{
    BallState ball_state(Point(0, 0.5), Vector(0, 0));

    auto friendly_robots = TestUtil::createMovingRobotStatesWithId(
        {Point(-3, 2.5), Point()}, {Vector(), Vector(4, 4)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<HaltTactic>();
    setTactic(1, tactic);

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            for (unsigned i = 0; i < 1000; i++)
            {
                robotHalt(world_ptr, yield);
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field_type, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(6));
}
