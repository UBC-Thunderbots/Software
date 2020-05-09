#include "software/simulation/physics/physics_world.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/world/field.h"

TEST(PhysicsSimulatorTest, test_world_does_not_change_if_time_step_is_zero)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world       = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
                                                        Timestamp::fromSeconds(0));
    world       = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
                                                     Timestamp::fromSeconds(0));

    PhysicsWorld physics_world(world);
    physics_world.stepSimulation(Duration::fromSeconds(0));
    World updated_world = physics_world.getWorld();

    EXPECT_EQ(world.ball(), updated_world.ball());
    EXPECT_EQ(world.field(), updated_world.field());
    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
        Point(1, 0.3), 1e-6));
    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
        Point(-0.5, -3), 1e-6));
    EXPECT_EQ(world.getMostRecentTimestamp(), updated_world.getMostRecentTimestamp());
}

TEST(PhysicsSimulatorTest, test_single_small_time_step)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(1, -0.5),
                                              Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
                                                        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
                                                     Timestamp::fromSeconds(0));

    PhysicsWorld physics_world(world);
    physics_world.stepSimulation(Duration::fromSeconds(0.01));
    World updated_world = physics_world.getWorld();

    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.01));
    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.01, -0.005), 1e-6));
    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.01));
    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
        Point(1, 0.3), 1e-6));
    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
        Point(-0.5, -3), 1e-6));
}

TEST(PhysicsSimulatorTest, test_several_consecutive_steps_of_varying_lengths)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setBallVelocity(world, Vector(1.0, -0.5),
                                              Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
                                                        Timestamp::fromSeconds(0));
    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
                                                     Timestamp::fromSeconds(0));
    PhysicsWorld physics_world(world);

    // very small step
    physics_world.stepSimulation(Duration::fromSeconds(0.005));
    World updated_world = physics_world.getWorld();
    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.005));
    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.005, -0.0025), 1e-6));
    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.005));
    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
        Point(1, 0.3), 1e-6));
    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
        Point(-0.5, -3), 1e-6));

    // medium step
    physics_world.stepSimulation(Duration::fromSeconds(0.1));
    updated_world = physics_world.getWorld();
    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.105));
    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.105, -0.0525), 1e-6));
    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.105));
    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
        Point(1, 0.3), 1e-6));
    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
        Point(-0.5, -3), 1e-6));

    // small step
    physics_world.stepSimulation(Duration::fromSeconds(0.01));
    updated_world = physics_world.getWorld();
    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.115));
    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.115, -0.0575), 1e-6));
    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.115));
    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
        Point(1, 0.3), 1e-6));
    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
        Point(-0.5, -3), 1e-6));
}
