#include "software/backend/simulation/physics_simulator.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/world/field.h"

TEST(PhysicsSimulatorTest, test_world_does_not_change_if_time_step_is_zero)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    PhysicsSimulator simulator(world);
    World updated_world = simulator.stepSimulation(Duration::fromSeconds(0));

    EXPECT_EQ(world.ball(), updated_world.ball());
    EXPECT_EQ(world.field(), updated_world.field());
    EXPECT_EQ(world.getMostRecentTimestamp(), updated_world.getMostRecentTimestamp());
}

TEST(PhysicsSimulatorTest, test_single_small_time_step)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(1, -0.5),
                                              Timestamp::fromSeconds(0));
    PhysicsSimulator simulator(world);
    World updated_world = simulator.stepSimulation(Duration::fromSeconds(0.1));

    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.1));
    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.1, -0.05), 1e-6));
    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.1));
}
