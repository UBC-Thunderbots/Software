#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/ai/hl/stp/play/example_play.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

class ExamplePlayTest : public SimulatedTest
{
};

TEST_F(ExamplePlayTest, test_example_play)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(4, 0)}, Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallPosition(world, Point(-1, 0), Timestamp::fromSeconds(0));

    std::vector<ValidationFunction> validation_functions = {
            // Wait for the robot to move in the direction of the ball
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while(world_ptr->friendlyTeam().getAllRobots().at(0).position().x() > 3.95) {
                    yield();
                }
            }
    };

    std::vector<ValidationFunction> continous_validation_functions = {};

    Util::MutableDynamicParameters->getMutableAIControlConfig()->mutableOverrideAIPlay()->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()->mutableCurrentAIPlay()->setValue(ExamplePlay::name);

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(5));
    EXPECT_TRUE(test_passed);
}

