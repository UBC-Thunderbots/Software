#include "software/simulation/simulator.h"

#include <gtest/gtest.h>

#include "software/primitive/move_primitive.h"
#include "software/primitive/primitive.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

TEST(SimulatorTest, test_simulation_step_updates_the_ball)
{
    // A sanity test to make sure stepping the simulation actually updates\
    // the state of the world

    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() =
        Ball(Point(0.4, 0), Vector(-1.3, 2.01), Timestamp::fromSeconds(0));

    Simulator simulator(world);
    simulator.stepSimulation(Duration::fromSeconds(0.1));
    World new_world = simulator.getWorld();
    Point p         = new_world.ball().position();
    EXPECT_NE(Point(0.4, 0), p);
}

TEST(SimulatorTest, test_simulate_robots_with_no_primitives)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    Simulator simulator(world);
    for (unsigned int i = 0; i < 60; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Robots have not been assigned primitives and so should not move
    World new_world                = simulator.getWorld();
    std::optional<Robot> new_robot = new_world.friendlyTeam().getRobotById(0);
    ASSERT_TRUE(new_robot);
    EXPECT_LT((new_robot->position() - Point(0, 0)).length(), 0.01);
}

TEST(SimulatorTest, test_simulate_single_robot_with_primitive)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    World world = ::Test::TestUtil::createBlankTestingWorld();
    // Move the ball away from (0, 0) so it doesn't interfere with the robot
    world.mutableBall() = Ball(Point(6, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    Robot robot(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot});

    std::unique_ptr<Primitive> move_primitive = std::make_unique<MovePrimitive>(
        0, Point(1, 0), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> primitives;
    primitives.emplace_back(std::move(move_primitive));
    auto primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
        std::move(primitives));

    Simulator simulator(world);
    simulator.setPrimitives(primitives_ptr);

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    World new_world                = simulator.getWorld();
    std::optional<Robot> new_robot = new_world.friendlyTeam().getRobotById(0);
    ASSERT_TRUE(new_robot);
    EXPECT_LT((new_robot->position() - Point(1, 0)).length(), 0.2);
}

TEST(SimulatorTest, test_simulate_multiple_robots_with_primitives)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    World world = ::Test::TestUtil::createBlankTestingWorld();
    // Move the ball away from (0, 0) so it doesn't interfere with the robots
    world.mutableBall() = Ball(Point(6, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    Robot robot_0(0, Point(0, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(-1, -1), Vector(0, 0), Angle::quarter(),
                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({robot_0, robot_1});

    std::unique_ptr<Primitive> move_primitive_0 = std::make_unique<MovePrimitive>(
        0, Point(1, 0), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutokickType::NONE);
    std::unique_ptr<Primitive> move_primitive_1 = std::make_unique<MovePrimitive>(
        1, Point(0, -2), Angle::half(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> primitives;
    primitives.emplace_back(std::move(move_primitive_0));
    primitives.emplace_back(std::move(move_primitive_1));
    auto primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
        std::move(primitives));

    Simulator simulator(world);
    simulator.setPrimitives(primitives_ptr);

    // Because simulation is very fast, and we don't want these tests to be fragile and
    // break due to subtle issues with control or firmware, we let this simulation run for
    // a long time to robots should settle at their final destinations before we perform
    // our assertions. All we care about is that everything worked together to ultimately
    // get them to their desired final states
    for (unsigned int i = 0; i < 600; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    World new_world = simulator.getWorld();

    std::optional<Robot> new_robot_0 = new_world.friendlyTeam().getRobotById(0);
    ASSERT_TRUE(new_robot_0);
    EXPECT_LT((new_robot_0->position() - Point(1, 0)).length(), 0.2);
    EXPECT_LT((new_robot_0->orientation().minDiff(Angle::zero())), Angle::fromDegrees(2));

    std::optional<Robot> new_robot_1 = new_world.friendlyTeam().getRobotById(1);
    ASSERT_TRUE(new_robot_1);
    EXPECT_LT((new_robot_1->position() - Point(0, -2)).length(), 0.2);
    EXPECT_LT((new_robot_1->orientation().minDiff(Angle::half())), Angle::fromDegrees(5));
}
