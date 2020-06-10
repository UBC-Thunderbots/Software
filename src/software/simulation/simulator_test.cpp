#include "software/simulation/simulator.h"

#include <gtest/gtest.h>

#include "software/primitive/move_primitive.h"
#include "software/primitive/primitive.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

// TODO: Add more specific tests for adding a ball when there is / isn't
// already a ball. Need to wait until the world output is replaced with
// proto in https://github.com/UBC-Thunderbots/Software/issues/1244

TEST(SimulatorTest, test_set_ball_state) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator.setBallState(ball_state);

    World world = simulator.getWorld();
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(ball_state, world.ball().currentState().ballState(), 1e-6));
}

TEST(SimulatorTest, test_remove_ball) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator.setBallState(ball_state);

    World world = simulator.getWorld();
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(ball_state, world.ball().currentState().ballState(), 1e-6));

    simulator.removeBall();
    world = simulator.getWorld();

    // TODO: Check the ball is actually removed once we have proto output
    // https://github.com/UBC-Thunderbots/Software/issues/1244
    // The World can't represent "no ball"
    BallState expected_ball_state(Point(0, 0), Vector(0, 0));
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(expected_ball_state, world.ball().currentState().ballState(), 1e-6));
}

TEST(SimualtorTest, add_zero_yellow_robots) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    simulator.addYellowRobots({});

    World world = simulator.getWorld();
    EXPECT_EQ(0, world.friendlyTeam().numRobots());
}

TEST(SimulatorTest, add_multiple_yellow_robots_with_valid_ids) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    RobotState robot_state3(Point(-2, -2), Vector(0, 4), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
            RobotStateWithId{.id = 2, .robot_state = robot_state2},
            RobotStateWithId{.id = 3, .robot_state = robot_state3},
    };
    simulator.addYellowRobots(states);

    World world = simulator.getWorld();
    EXPECT_EQ(3, world.friendlyTeam().numRobots());
}

TEST(SimulatorTest, add_yellow_robots_with_duplicate_ids) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
            RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(simulator.addYellowRobots(states), std::runtime_error);
}

TEST(SimulatorTest, add_yellow_robots_with_ids_that_already_exist_in_the_simulation) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(simulator.addYellowRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(simulator.addYellowRobots(states2), std::runtime_error);
}

TEST(SimualtorTest, add_zero_blue_robots) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    simulator.addBlueRobots({});

    World world = simulator.getWorld();
    EXPECT_EQ(0, world.enemyTeam().numRobots());
}

TEST(SimulatorTest, add_multiple_blue_robots_with_valid_ids) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    RobotState robot_state3(Point(-2, -2), Vector(0, 4), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
            RobotStateWithId{.id = 2, .robot_state = robot_state2},
            RobotStateWithId{.id = 3, .robot_state = robot_state3},
    };
    simulator.addBlueRobots(states);

    World world = simulator.getWorld();
    EXPECT_EQ(3, world.enemyTeam().numRobots());
}

TEST(SimulatorTest, add_blue_robots_with_duplicate_ids) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
            RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(simulator.addBlueRobots(states), std::runtime_error);
}

TEST(SimulatorTest, add_blue_robots_with_ids_that_already_exist_in_the_simulation) {
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(simulator.addBlueRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(simulator.addBlueRobots(states2), std::runtime_error);
}

TEST(SimulatorTest, test_simulation_step_updates_the_ball)
{
    // A sanity test to make sure stepping the simulation actually updates
    // the state of the world

    Simulator simulator(::Test::TestUtil::createSSLDivBField());
    simulator.setBallState(BallState(Point(0.4, 0), Vector(-1.3, 2.01)));

    simulator.stepSimulation(Duration::fromSeconds(0.1));
    World world = simulator.getWorld();

    Point ball_position = world.ball().position();
    EXPECT_NE(Point(0.4, 0), ball_position);
    Vector ball_velocity = world.ball().velocity();
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(Vector(-1.3, 2.01), ball_velocity, 1e-4));
}

TEST(SimulatorTest, test_simulate_yellow_robots_with_no_primitives)
{
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addYellowRobots(states);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Robots have not been assigned primitives and so should not move
    World world = simulator.getWorld();
    std::optional<Robot> robot = world.friendlyTeam().getRobotById(1);
    ASSERT_TRUE(robot);
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(robot->position(), Point(0, 0), 0.01));
}

TEST(SimulatorTest, test_simulate_single_yellow_robot_with_primitive)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addYellowRobots(states);

    std::unique_ptr<Primitive> move_primitive = std::make_unique<MovePrimitive>(
        1, Point(1, 0), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
        AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> primitives;
    primitives.emplace_back(std::move(move_primitive));
    auto primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
        std::move(primitives));
    simulator.setYellowRobotPrimitives(primitives_ptr);

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    World world = simulator.getWorld();
    std::optional<Robot> robot = world.friendlyTeam().getRobotById(1);
    ASSERT_TRUE(robot);
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(robot->position(), Point(1, 0), 0.2));
}

TEST(SimulatorTest, test_simulate_blue_robots_with_no_primitives)
{
    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addBlueRobots(states);

    for (unsigned int i = 0; i < 60; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // Robots have not been assigned primitives and so should not move
    World world = simulator.getWorld();
    std::optional<Robot> robot = world.enemyTeam().getRobotById(1);
    ASSERT_TRUE(robot);
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(robot->position(), Point(0, 0), 0.01));
}

TEST(SimulatorTest, test_simulate_single_blue_robot_with_primitive)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
            RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addBlueRobots(states);

    std::unique_ptr<Primitive> move_primitive = std::make_unique<MovePrimitive>(
            1, Point(1, 0), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
            AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> primitives;
    primitives.emplace_back(std::move(move_primitive));
    auto primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(primitives));
    simulator.setBlueRobotPrimitives(primitives_ptr);

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    World world = simulator.getWorld();
    std::optional<Robot> robot = world.enemyTeam().getRobotById(1);
    ASSERT_TRUE(robot);
    EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(robot->position(), Point(1, 0), 0.2));
}

TEST(SimulatorTest, test_simulate_multiple_blue_and_yellow_robots_with_primitives)
{
    // Simulate multiple robots with primitives to sanity check that everything is connected
    // properly and we can properly simulate multiple instances of the robot firmware at once.
    // We use the MovePrimitve because it is very commonly used and so unlikely to be
    // significantly changed or removed, and its behaviour is easy to validate

    Simulator simulator(::Test::TestUtil::createSSLDivBField());

    RobotState blue_robot_state1(Point(-1, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    RobotState blue_robot_state2(Point(-2, 1), Vector(0, 0), Angle::quarter(),
                                 AngularVelocity::zero());
    std::vector<RobotStateWithId> blue_robot_states = {
            RobotStateWithId{.id = 1, .robot_state = blue_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = blue_robot_state2},
    };
    simulator.addBlueRobots(blue_robot_states);

    RobotState yellow_robot_state1(Point(1, 1.5), Vector(0, 0), Angle::half(),
                                 AngularVelocity::zero());
    RobotState yellow_robot_state2(Point(2.5, -1), Vector(0, 0), Angle::threeQuarter(),
                                 AngularVelocity::zero());
    std::vector<RobotStateWithId> yellow_robot_states = {
            RobotStateWithId{.id = 1, .robot_state = yellow_robot_state1},
            RobotStateWithId{.id = 2, .robot_state = yellow_robot_state2},
    };
    simulator.addYellowRobots(yellow_robot_states);

    std::unique_ptr<Primitive> blue_move_primitive1 = std::make_unique<MovePrimitive>(
            1, Point(-1, -1), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
            AutokickType::NONE);
    std::unique_ptr<Primitive> blue_move_primitive2 = std::make_unique<MovePrimitive>(
            2, Point(-3, 0), Angle::half(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
            AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> blue_robot_primitives;
    blue_robot_primitives.emplace_back(std::move(blue_move_primitive1));
    blue_robot_primitives.emplace_back(std::move(blue_move_primitive2));
    auto blue_primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(blue_robot_primitives));
    simulator.setBlueRobotPrimitives(blue_primitives_ptr);

    std::unique_ptr<Primitive> yellow_move_primitive1 = std::make_unique<MovePrimitive>(
            1, Point(1, 1), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
            AutokickType::NONE);
    std::unique_ptr<Primitive> yellow_move_primitive2 = std::make_unique<MovePrimitive>(
            2, Point(3, -2), Angle::zero(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
            AutokickType::NONE);
    std::vector<std::unique_ptr<Primitive>> yellow_robot_primitives;
    yellow_robot_primitives.emplace_back(std::move(yellow_move_primitive1));
    yellow_robot_primitives.emplace_back(std::move(yellow_move_primitive2));
    auto yellow_primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(yellow_robot_primitives));
    simulator.setYellowRobotPrimitives(yellow_primitives_ptr);

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    World world = simulator.getWorld();

    // TODO: These tests are currently very lenient, and don't test final velocities.
    //  This is because they currently rely on controller dynamics, and the existing
    //  bang-bang controller tends to overshoot with the current physics damping constants.
    //  In order to help decouple these tests from the controller / damping, the test
    //  tolerances are larger for now. They should be tightened again when the new
    //  controller is implemented.
    //  https://github.com/UBC-Thunderbots/Software/issues/1187

    std::optional<Robot> yellow_robot_1 = world.friendlyTeam().getRobotById(1);
    ASSERT_TRUE(yellow_robot_1);
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Point(1, 1), yellow_robot_1->position(), 0.2)));
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Angle::zero(), yellow_robot_1->orientation(), Angle::fromDegrees(10))));

    std::optional<Robot> yellow_robot_2 = world.friendlyTeam().getRobotById(2);
    ASSERT_TRUE(yellow_robot_2);
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Point(3, -2), yellow_robot_2->position(), 0.2)));
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Angle::zero(), yellow_robot_2->orientation(), Angle::fromDegrees(10))));

    std::optional<Robot> blue_robot_1 = world.enemyTeam().getRobotById(1);
    ASSERT_TRUE(blue_robot_1);
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Point(-1, -1), blue_robot_1->position(), 0.3)));
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Angle::zero(), blue_robot_1->orientation(), Angle::fromDegrees(10))));

    std::optional<Robot> blue_robot_2 = world.enemyTeam().getRobotById(2);
    ASSERT_TRUE(blue_robot_2);
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Point(-3, 0), blue_robot_2->position(), 0.3)));
    EXPECT_TRUE((::Test::TestUtil::equalWithinTolerance(Angle::half(), blue_robot_2->orientation(), Angle::fromDegrees(10))));
}
