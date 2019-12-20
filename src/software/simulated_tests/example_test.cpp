#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/test_util/test_util.h"
#include "software/util/time/duration.h"
#include "software/world/world.h"

TEST_F(SimulatedTest, example_simulated_test)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));

    bool test_succeeded = backend->runSimulation(world, Duration::fromSeconds(10));

    // Currently the simulation always times out because validation is not implemented yet
    ASSERT_FALSE(test_succeeded);
}
