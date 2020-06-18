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
};

TEST_F(NewPlayTest, FooTest)
{
    enableVisualizer();
    setBallState(BallState(Point(-0.8, 0), Vector(0, 0)));
    addFriendlyRobots({
        RobotStateWithId{.id = 0, .robot_state = RobotState(Point(-4, 3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
        RobotStateWithId{.id = 1, .robot_state = RobotState(Point(0, 3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
//        RobotStateWithId{.id = 2, .robot_state = RobotState(Point(4, 3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
//        RobotStateWithId{.id = 3, .robot_state = RobotState(Point(-4, -3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
//        RobotStateWithId{.id = 4, .robot_state = RobotState(Point(0, -3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
//        RobotStateWithId{.id = 5, .robot_state = RobotState(Point(4, -3), Vector(0, 0), Angle::zero(), AngularVelocity::zero())},
    });
    setPlay(ExamplePlay::name);

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while(true) {
//                std::cout << world_ptr->getMostRecentTimestamp() << std::endl;
                yield();
            };
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    runTest(validation_functions, continous_validation_functions, Duration::fromSeconds(10));
}
