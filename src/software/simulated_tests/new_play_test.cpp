#include "software/ai/hl/stp/play/example_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

class NewPlayTest : public SimulatedTest
{
//    NewPlayTest() : SimulatedTest() {}
};

TEST_F(NewPlayTest, FooTest)
{
    simulator.setBallState(BallState(Point(-0.8, 0), Vector(0, 0)));
    simulator.addYellowRobots({
        RobotStateWithId{.id = 0, .robot_state = RobotState(Point(4, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
    });

//    World world = ::TestUtil::createBlankTestingWorld();
//    world =
//        ::TestUtil::setFriendlyRobotPositions(world,
//                                              {Point(4, 0), Point(0.5, 0), Point(-3, 1),
//                                               Point(-1, -3), Point(2, 0), Point(3.5, 3)},
//                                              Timestamp::fromSeconds(0));
//    world.mutableBall() = Ball(Point(-0.8, 0), Vector(0, 0), Timestamp::fromSeconds(0));

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while(true) {yield();};
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    Util::MutableDynamicParameters->getMutableAIControlConfig()
            ->mutableOverrideAIPlay()
            ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
            ->mutableCurrentAIPlay()
            ->setValue(ExamplePlay::name);

    runTest(validation_functions, continous_validation_functions, Duration::fromSeconds(10));


//
//    backend->startSimulation(world);
//    bool test_passed = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(8));
//    EXPECT_TRUE(test_passed);
}
