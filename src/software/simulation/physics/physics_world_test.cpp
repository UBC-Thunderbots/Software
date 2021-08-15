#include "software/simulation/physics/physics_world.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/world/field.h"

class PhysicsWorldTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        simulator_config = std::make_shared<const SimulatorConfig>();
        physics_world = std::make_shared<PhysicsWorld>(Field::createSSLDivisionBField(),
                                                       simulator_config);
    }

    std::shared_ptr<const SimulatorConfig> simulator_config;
    std::shared_ptr<PhysicsWorld> physics_world;
};

// A matcher used to compare RobotStateWitId_t structs with enough tolerance
// for floating point error introduced by PhysicsWorld and the underlying
// physics library
MATCHER(RobotStateWithIdEq, "Robot State with Id Equal")
{
    return TestUtil::equalWithinTolerance(::std::get<0>(arg), ::std::get<1>(arg), 1e-6,
                                          Angle::fromDegrees(0.1));
}

TEST_F(PhysicsWorldTest, default_construct_physics_world)
{
    EXPECT_EQ(Field::createSSLDivisionBField(), physics_world->getField());
    EXPECT_FALSE(physics_world->getBallState().has_value());
    EXPECT_TRUE(physics_world->getYellowRobotStates().empty());
    EXPECT_TRUE(physics_world->getBlueRobotStates().empty());
    EXPECT_EQ(Timestamp::fromSeconds(0), physics_world->getTimestamp());
}

TEST_F(PhysicsWorldTest, test_set_ball_state_when_no_ball_exists)
{
    BallState ball_state(Point(1, -2), Vector(0, 0.5));
    physics_world->setBallState(ball_state);

    ASSERT_TRUE(physics_world->getBallState());
    EXPECT_EQ(ball_state, physics_world->getBallState().value());
}

TEST_F(PhysicsWorldTest, test_set_ball_state_when_ball_already_exists)
{
    BallState ball_state1(Point(1, -2), Vector(0, 0.5));
    physics_world->setBallState(ball_state1);

    BallState ball_state2(Point(0, 3), Vector(2, 1));
    physics_world->setBallState(ball_state2);

    ASSERT_TRUE(physics_world->getBallState());
    EXPECT_EQ(ball_state2, physics_world->getBallState().value());
}

TEST_F(PhysicsWorldTest, test_add_zero_yellow_robots)
{
    physics_world->addYellowRobots({});

    EXPECT_TRUE(physics_world->getYellowRobotStates().empty());
}

TEST_F(PhysicsWorldTest, test_add_single_yellow_robot_with_valid_id)
{
    RobotState robot_state(Point(1, 0), Vector(0, 0), Angle::quarter(),
                           AngularVelocity::half());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world->addYellowRobots(states);

    ASSERT_EQ(1, physics_world->getYellowRobotStates().size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world->getYellowRobotStates()));
}

TEST_F(PhysicsWorldTest, test_add_multiple_yellow_robot_with_valid_ids)
{
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
    physics_world->addYellowRobots(states);

    auto yellow_robot_states = physics_world->getYellowRobotStates();
    ASSERT_EQ(3, yellow_robot_states.size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world->getYellowRobotStates()));
}

TEST_F(PhysicsWorldTest, test_add_yellow_robots_with_duplicate_ids)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(physics_world->addYellowRobots(states), std::runtime_error);
}

TEST_F(PhysicsWorldTest, test_add_yellow_robots_to_physics_world_with_existing_ids)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(physics_world->addYellowRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(physics_world->addYellowRobots(states2), std::runtime_error);
}

TEST_F(PhysicsWorldTest, test_add_zero_blue_robots)
{
    physics_world->addBlueRobots({});

    EXPECT_TRUE(physics_world->getBlueRobotStates().empty());
}

TEST_F(PhysicsWorldTest, test_add_single_blue_robot_with_valid_id)
{
    RobotState robot_state(Point(1, 0), Vector(0, 0), Angle::quarter(),
                           AngularVelocity::half());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world->addBlueRobots(states);

    ASSERT_EQ(1, physics_world->getBlueRobotStates().size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world->getBlueRobotStates()));
}

TEST_F(PhysicsWorldTest, test_add_multiple_blue_robot_with_valid_ids)
{
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
    physics_world->addBlueRobots(states);

    auto blue_robot_states = physics_world->getBlueRobotStates();
    ASSERT_EQ(3, blue_robot_states.size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world->getBlueRobotStates()));
}

TEST_F(PhysicsWorldTest, test_add_blue_robots_with_duplicate_ids)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(physics_world->addBlueRobots(states), std::runtime_error);
}

TEST_F(PhysicsWorldTest, test_add_blue_robots_to_physics_world_with_existing_ids)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(physics_world->addBlueRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(physics_world->addBlueRobots(states2), std::runtime_error);
}

TEST_F(PhysicsWorldTest, test_remove_robot_with_invalid_ptr)
{
    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());

    physics_world->removeRobot(std::weak_ptr<PhysicsRobot>());

    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());
}

TEST_F(PhysicsWorldTest, remove_existing_yellow_team_robot)
{
    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    physics_world->addYellowRobots(states1);

    auto yellow_robots = physics_world->getYellowPhysicsRobots();
    ASSERT_FALSE(yellow_robots.empty());
    auto robot_to_remove = yellow_robots.at(0);

    physics_world->removeRobot(robot_to_remove);

    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());
}

TEST_F(PhysicsWorldTest, remove_existing_blue_team_robot)
{
    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    physics_world->addBlueRobots(states1);

    auto blue_robots = physics_world->getBluePhysicsRobots();
    ASSERT_FALSE(blue_robots.empty());
    auto robot_to_remove = blue_robots.at(0);

    physics_world->removeRobot(robot_to_remove);

    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());
}

TEST_F(PhysicsWorldTest, remove_existing_robot_twice)
{
    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    physics_world->addBlueRobots(states1);

    auto blue_robots = physics_world->getBluePhysicsRobots();
    ASSERT_FALSE(blue_robots.empty());
    auto robot_to_remove = blue_robots.at(0);

    physics_world->removeRobot(robot_to_remove);
    physics_world->removeRobot(robot_to_remove);

    EXPECT_TRUE(physics_world->getBluePhysicsRobots().empty());
    EXPECT_TRUE(physics_world->getYellowPhysicsRobots().empty());
}

TEST_F(PhysicsWorldTest, get_available_yellow_robot_ids_with_no_existing_robots)
{
    EXPECT_EQ(0, physics_world->getAvailableYellowRobotId());
}

TEST_F(PhysicsWorldTest, get_available_yellow_robot_ids_with_existing_robots)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 2, .robot_state = robot_state2},
    };
    physics_world->addYellowRobots(states);

    EXPECT_EQ(1, physics_world->getAvailableYellowRobotId());
}

TEST_F(PhysicsWorldTest, get_available_blue_robot_ids_with_no_existing_robots)
{
    EXPECT_EQ(0, physics_world->getAvailableBlueRobotId());
}

TEST_F(PhysicsWorldTest, get_available_blue_robot_ids_with_existing_robots)
{
    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 2, .robot_state = robot_state2},
    };
    physics_world->addBlueRobots(states);

    EXPECT_EQ(1, physics_world->getAvailableBlueRobotId());
}

TEST_F(PhysicsWorldTest, test_world_does_not_change_if_time_step_is_zero)
{
    BallState ball_state(Point(0, 1), Vector(2, -3));

    RobotState yellow_robot_state(Point(1, 0), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = yellow_robot_state},
    };

    RobotState blue_robot_state(Point(0, 0), Vector(-2, 1), Angle::half(),
                                AngularVelocity::quarter());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = blue_robot_state},
    };


    physics_world->setBallState(ball_state);
    physics_world->addYellowRobots(yellow_robot_states);
    physics_world->addBlueRobots(blue_robot_states);

    physics_world->stepSimulation(Duration::fromSeconds(0));

    auto updated_ball_state = physics_world->getBallState();
    std::vector<RobotStateWithId> updated_yellow_robot_states =
        physics_world->getYellowRobotStates();
    std::vector<RobotStateWithId> updated_blue_robot_states =
        physics_world->getBlueRobotStates();

    ASSERT_TRUE(updated_ball_state);

    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(ball_state, updated_ball_state.value(), 1e-6));
    EXPECT_THAT(
        yellow_robot_states,
        ::testing::UnorderedPointwise(RobotStateWithIdEq(), updated_yellow_robot_states));
    EXPECT_THAT(blue_robot_states, ::testing::UnorderedPointwise(
                                       RobotStateWithIdEq(), updated_blue_robot_states));
    EXPECT_EQ(physics_world->getField(), Field::createSSLDivisionBField());
}

TEST_F(PhysicsWorldTest, test_single_small_time_step)
{
    BallState ball_state(Point(0, 1), Vector(2, -3));

    RobotState yellow_robot_state(Point(1, 0), Vector(3, 0), Angle::quarter(),
                                  AngularVelocity::half());
    std::vector<RobotStateWithId> yellow_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = yellow_robot_state},
    };

    RobotState blue_robot_state(Point(0, 0), Vector(-2, 1), Angle::half(),
                                AngularVelocity::quarter());
    std::vector<RobotStateWithId> blue_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = blue_robot_state},
    };


    physics_world->setBallState(ball_state);
    physics_world->addYellowRobots(yellow_robot_states);
    physics_world->addBlueRobots(blue_robot_states);

    physics_world->stepSimulation(Duration::fromSeconds(1.0 / 60.0));

    auto updated_ball_state = physics_world->getBallState();
    std::vector<RobotStateWithId> updated_yellow_robot_states =
        physics_world->getYellowRobotStates();
    std::vector<RobotStateWithId> updated_blue_robot_states =
        physics_world->getBlueRobotStates();

    ASSERT_TRUE(updated_ball_state);

    BallState expected_ball_state(Point(0.0 + 2.0 / 60, 1 - 3.0 / 60.0), Vector(2, -3));
    RobotState expected_yellow_robot_state(
        Point(1 + 3.0 / 60.0, 0), Vector(3, 0),
        Angle::quarter() + AngularVelocity::half() / 60.0, AngularVelocity::half());
    std::vector<RobotStateWithId> expected_yellow_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = expected_yellow_robot_state},
    };

    RobotState expected_blue_robot_state(
        Point(0 - 2.0 / 60.0, 1.0 / 60.0), Vector(-2, 1),
        Angle::half() + AngularVelocity::quarter() / 60.0, AngularVelocity::quarter());
    std::vector<RobotStateWithId> expected_blue_robot_states = {
        RobotStateWithId{.id = 0, .robot_state = expected_blue_robot_state},
    };

    EXPECT_TRUE(TestUtil::equalWithinTolerance(expected_ball_state,
                                               updated_ball_state.value(), 1e-4));
    EXPECT_THAT(expected_yellow_robot_states,
                ::testing::Not(::testing::UnorderedPointwise(
                    RobotStateWithIdEq(), updated_yellow_robot_states)));
    EXPECT_THAT(expected_blue_robot_states,
                ::testing::Not(::testing::UnorderedPointwise(RobotStateWithIdEq(),
                                                             updated_blue_robot_states)));
    EXPECT_EQ(physics_world->getField(), Field::createSSLDivisionBField());
}

TEST_F(PhysicsWorldTest, test_get_robot_at_position_without_robot)
{
    auto result = physics_world->getRobotAtPosition(Point(1, 1));
    EXPECT_FALSE(result.lock());
}

TEST_F(PhysicsWorldTest, test_get_robot_at_position_with_exact_robot_position)
{
    RobotState robot_state(Point(1, 0), Vector(0, 0), Angle::quarter(),
                           AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world->addYellowRobots(states);

    auto result = physics_world->getRobotAtPosition(Point(1, 0));
    ASSERT_TRUE(result.lock());
    auto physics_robot = result.lock();
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(physics_robot->position(), Point(1, 0), 1e-3));
}

TEST_F(PhysicsWorldTest, test_get_robot_at_position_near_edge_of_robot)
{
    RobotState robot_state(Point(1, 2), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world->addBlueRobots(states);

    auto result =
        physics_world->getRobotAtPosition(Point(1, 2 + ROBOT_MAX_RADIUS_METERS * 0.9));
    ASSERT_TRUE(result.lock());
    auto physics_robot = result.lock();
    EXPECT_TRUE(
        ::TestUtil::equalWithinTolerance(physics_robot->position(), Point(1, 2), 1e-3));
}
