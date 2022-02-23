#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/intent/stop_intent.h"
#include "software/simulated_tests/simulated_er_force_sim_tactic_test_fixture.h"
#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"
#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"

class StopTacticTest : public SimulatedErForceSimTacticTestFixture
{
   protected:
    FieldType field_type = FieldType::DIV_B;
    Field field          = Field::createField(field_type);
};


TEST(StopTacticCostTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    StopTactic tactic = StopTactic(false);

    // We always expect the cost to be 0.5, because the StopTactic prefers all robots
    // equally
    EXPECT_EQ(0.5, tactic.calculateRobotCost(robot, world));
}

TEST_F(StopTacticTest, robot_already_stopped)
{
    BallState ball_state(Point(0, 0.5), Vector(0, 0));

    auto friendly_robots = TestUtil::createMovingRobotStatesWithId(
        {Point(-3, 2.5), Point()}, {Vector(), Vector()});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<StopTactic>(false);
    setTactic(tactic);
    setFriendlyRobotId(1);

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

TEST_F(StopTacticTest, robot_start_moving)
{
    BallState ball_state(Point(0, 0.5), Vector(0, 0));

    auto friendly_robots = TestUtil::createMovingRobotStatesWithId(
        {Point(-3, 2.5), Point()}, {Vector(), Vector(4, 4)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});

    auto tactic = std::make_shared<StopTactic>(false);
    setTactic(tactic);
    setFriendlyRobotId(1);

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
