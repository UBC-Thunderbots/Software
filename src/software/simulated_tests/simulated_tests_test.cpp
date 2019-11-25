#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/test_util/test_util.h"
#include "software/util/time/duration.h"
#include "software/backend/simulation/validation/validation_function.h"
#include "software/world/world.h"

TEST_F(SimulatedTest, test_single_validation_function_passes_before_timeout)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));
    std::vector<ValidationFunction> validation_functions = {
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while(world_ptr->ball().position().x() < 3) {
                    yield();
                }
            }
    };

    bool test_passed = backend->runSimulation(validation_functions, world, Duration::fromSeconds(1));

    ASSERT_TRUE(test_passed);
}

TEST_F(SimulatedTest, test_single_validation_function_fails_if_it_times_out)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));
    std::vector<ValidationFunction> validation_functions = {
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                while(world_ptr->ball().position().x() < 3) {
                    yield();
                }
            }
    };

    bool test_passed = backend->runSimulation(validation_functions, world, Duration::fromSeconds(0.5));

    ASSERT_FALSE(test_passed);
}
