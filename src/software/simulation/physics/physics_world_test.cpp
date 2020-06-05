#include "software/simulation/physics/physics_world.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/world/field.h"

// A matcher used to compare RobotStateWitId_t structs with enough tolernace
// for floating point error introduced by PhysicsWorld and the underlying
// physics libraru
MATCHER(RobotStateWithIdEq, "Robot State with Id Equal")
{
    return ::Test::TestUtil::equalWithinTolerance(::std::get<0>(arg), ::std::get<1>(arg),
                                                  1e-6, Angle::fromDegrees(0.1));
}

TEST(PhysicsWorldTest, default_construct_physics_world)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    EXPECT_EQ(::Test::TestUtil::createSSLDivBField(), physics_world.getField());
    EXPECT_FALSE(physics_world.getBallState().has_value());
    EXPECT_TRUE(physics_world.getYellowRobotStates().empty());
    EXPECT_TRUE(physics_world.getBlueRobotStates().empty());
    EXPECT_EQ(Timestamp::fromSeconds(0), physics_world.getTimestamp());
}

TEST(PhysicsWorldTest, test_set_field_with_valid_field)
{
    Field field1(1, 1, 1, 1, 1, 1, 1);
    Field field2(2, 2, 2, 2, 2, 2, 2);

    PhysicsWorld physics_world(field1);

    physics_world.setField(field1);
    EXPECT_EQ(field1, physics_world.getField());

    physics_world.setField(field2);
    EXPECT_EQ(field2, physics_world.getField());
}

TEST(PhysicsWorldTest, test_set_field_with_invalid)
{
    Field valid_field = ::Test::TestUtil::createSSLDivBField();
    PhysicsWorld physics_world(valid_field);

    physics_world.setField(Field());
    EXPECT_EQ(valid_field, physics_world.getField());
}

TEST(PhysicsWorldTest, test_set_ball_state_when_no_ball_exists)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    BallState ball_state(Point(1, -2), Vector(0, 0.5));
    physics_world.setBallState(ball_state);

    ASSERT_TRUE(physics_world.getBallState());
    EXPECT_EQ(ball_state, physics_world.getBallState().value());
}

TEST(PhysicsWorldTest, test_set_ball_state_when_ball_already_exists)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    BallState ball_state1(Point(1, -2), Vector(0, 0.5));
    physics_world.setBallState(ball_state1);

    BallState ball_state2(Point(0, 3), Vector(2, 1));
    physics_world.setBallState(ball_state2);

    ASSERT_TRUE(physics_world.getBallState());
    EXPECT_EQ(ball_state2, physics_world.getBallState().value());
}

TEST(PhysicsWorldTest, test_remove_existing_ball)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    BallState ball_state(Point(1, -2), Vector(0, 0.5));
    physics_world.setBallState(ball_state);

    EXPECT_TRUE(physics_world.getBallState());

    physics_world.removeBall();

    EXPECT_FALSE(physics_world.getBallState());
}

TEST(PhysicsWorldTest, test_remove_nonexistant_ball)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    physics_world.removeBall();
    EXPECT_FALSE(physics_world.getBallState());
}

TEST(PhysicsWorldTest, test_add_zero_yellow_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    physics_world.addYellowRobots({});

    EXPECT_TRUE(physics_world.getYellowRobotStates().empty());
}

TEST(PhysicsWorldTest, test_add_single_yellow_robot_with_valid_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state(Point(1, 0), Vector(0, 0), Angle::quarter(),
                           AngularVelocity::half());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world.addYellowRobots(states);

    ASSERT_EQ(1, physics_world.getYellowRobotStates().size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world.getYellowRobotStates()));
}

TEST(PhysicsWorldTest, test_add_multiple_yellow_robot_with_valid_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

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
    physics_world.addYellowRobots(states);

    auto yellow_robot_states = physics_world.getYellowRobotStates();
    ASSERT_EQ(3, yellow_robot_states.size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world.getYellowRobotStates()));
}

TEST(PhysicsWorldTest, test_add_yellow_robots_with_duplicate_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(physics_world.addYellowRobots(states), std::runtime_error);
}

TEST(PhysicsWorldTest, test_add_yellow_robots_to_physics_world_with_existing_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(physics_world.addYellowRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(physics_world.addYellowRobots(states2), std::runtime_error);
}

TEST(PhysicsWorldTest, test_add_zero_blue_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    physics_world.addBlueRobots({});

    EXPECT_TRUE(physics_world.getBlueRobotStates().empty());
}

TEST(PhysicsWorldTest, test_add_single_blue_robot_with_valid_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state(Point(1, 0), Vector(0, 0), Angle::quarter(),
                           AngularVelocity::half());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state}};
    physics_world.addBlueRobots(states);

    ASSERT_EQ(1, physics_world.getBlueRobotStates().size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world.getBlueRobotStates()));
}

TEST(PhysicsWorldTest, test_add_multiple_blue_robot_with_valid_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

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
    physics_world.addBlueRobots(states);

    auto blue_robot_states = physics_world.getBlueRobotStates();
    ASSERT_EQ(3, blue_robot_states.size());
    EXPECT_THAT(states, ::testing::UnorderedPointwise(
                            RobotStateWithIdEq(), physics_world.getBlueRobotStates()));
}

TEST(PhysicsWorldTest, test_add_blue_robots_with_duplicate_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(physics_world.addBlueRobots(states), std::runtime_error);
}

TEST(PhysicsWorldTest, test_add_blue_robots_to_physics_world_with_existing_ids)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(physics_world.addBlueRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_THROW(physics_world.addBlueRobots(states2), std::runtime_error);
}

TEST(PhysicsWorldTest, get_available_yellow_robot_ids_with_no_existing_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    std::optional<unsigned int> id = physics_world.getAvailableYellowRobotId();
    ASSERT_TRUE(id);
    EXPECT_EQ(0, id.value());
}

TEST(PhysicsWorldTest, get_available_yellow_robot_ids_with_existing_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 2, .robot_state = robot_state2},
    };
    physics_world.addYellowRobots(states);

    std::optional<unsigned int> id = physics_world.getAvailableYellowRobotId();
    ASSERT_TRUE(id);
    EXPECT_EQ(1, id.value());
}

TEST(PhysicsWorldTest, get_available_blue_robot_ids_with_no_existing_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    std::optional<unsigned int> id = physics_world.getAvailableBlueRobotId();
    ASSERT_TRUE(id);
    EXPECT_EQ(0, id.value());
}

TEST(PhysicsWorldTest, get_available_blue_robot_ids_with_existing_robots)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 2, .robot_state = robot_state2},
    };
    physics_world.addBlueRobots(states);

    std::optional<unsigned int> id = physics_world.getAvailableBlueRobotId();
    ASSERT_TRUE(id);
    EXPECT_EQ(1, id.value());
}

TEST(PhysicsWorldTest, is_yellow_robot_id_available_with_available_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    EXPECT_TRUE(physics_world.isYellowRobotIdAvailable(0));
}

TEST(PhysicsWorldTest, is_yellow_robot_id_available_with_unavailable_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state},
    };
    physics_world.addYellowRobots(states);

    EXPECT_FALSE(physics_world.isYellowRobotIdAvailable(0));
}

TEST(PhysicsWorldTest, is_blue_robot_id_available_with_available_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    EXPECT_TRUE(physics_world.isBlueRobotIdAvailable(0));
}

TEST(PhysicsWorldTest, is_blue_robot_id_available_with_unavailable_id)
{
    PhysicsWorld physics_world(::Test::TestUtil::createSSLDivBField());

    RobotState robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state},
    };
    physics_world.addBlueRobots(states);

    EXPECT_FALSE(physics_world.isBlueRobotIdAvailable(0));
}

// TEST(PhysicsSimulatorTest, test_world_does_not_change_if_time_step_is_zero)
//{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world       = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
//                                                        Timestamp::fromSeconds(0));
//    world       = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
//                                                     Timestamp::fromSeconds(0));
//
//    PhysicsWorld physics_world(world);
//    physics_world.stepSimulation(Duration::fromSeconds(0));
//    World updated_world = physics_world.getWorld();
//
//    EXPECT_EQ(world.ball(), updated_world.ball());
//    EXPECT_EQ(world.field(), updated_world.field());
//    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
//        Point(1, 0.3), 1e-6));
//    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
//        Point(-0.5, -3), 1e-6));
//    EXPECT_EQ(world.getMostRecentTimestamp(), updated_world.getMostRecentTimestamp());
//}
//
// TEST(PhysicsSimulatorTest, test_single_small_time_step)
//{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world =
//        ::Test::TestUtil::setBallPosition(world, Point(0, 0),
//        Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setBallVelocity(world, Vector(1, -0.5),
//                                              Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
//                                                        Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
//                                                     Timestamp::fromSeconds(0));
//
//    PhysicsWorld physics_world(world);
//    physics_world.stepSimulation(Duration::fromSeconds(0.01));
//    World updated_world = physics_world.getWorld();
//
//    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.01));
//    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.01, -0.005), 1e-6));
//    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
//    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.01));
//    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
//        Point(1, 0.3), 1e-6));
//    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
//        Point(-0.5, -3), 1e-6));
//}
//
// TEST(PhysicsSimulatorTest, test_several_consecutive_steps_of_varying_lengths)
//{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world =
//        ::Test::TestUtil::setBallPosition(world, Point(0, 0),
//        Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setBallVelocity(world, Vector(1.0, -0.5),
//                                              Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {Point(1, 0.3)},
//                                                        Timestamp::fromSeconds(0));
//    world = ::Test::TestUtil::setEnemyRobotPositions(world, {Point(-0.5, -3)},
//                                                     Timestamp::fromSeconds(0));
//    PhysicsWorld physics_world(world);
//
//    // very small step
//    physics_world.stepSimulation(Duration::fromSeconds(0.005));
//    World updated_world = physics_world.getWorld();
//    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.005));
//    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.005, -0.0025), 1e-6));
//    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
//    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(),
//    Timestamp::fromSeconds(0.005));
//    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
//        Point(1, 0.3), 1e-6));
//    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
//        Point(-0.5, -3), 1e-6));
//
//    // medium step
//    physics_world.stepSimulation(Duration::fromSeconds(0.1));
//    updated_world = physics_world.getWorld();
//    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.105));
//    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.105, -0.0525), 1e-6));
//    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
//    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(),
//    Timestamp::fromSeconds(0.105));
//    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
//        Point(1, 0.3), 1e-6));
//    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
//        Point(-0.5, -3), 1e-6));
//
//    // small step
//    physics_world.stepSimulation(Duration::fromSeconds(0.01));
//    updated_world = physics_world.getWorld();
//    EXPECT_EQ(updated_world.getMostRecentTimestamp(), Timestamp::fromSeconds(0.115));
//    EXPECT_TRUE(updated_world.ball().position().isClose(Point(0.115, -0.0575), 1e-6));
//    EXPECT_EQ(updated_world.ball().velocity(), Vector(1, -0.5));
//    EXPECT_EQ(updated_world.ball().lastUpdateTimestamp(),
//    Timestamp::fromSeconds(0.115));
//    ASSERT_EQ(updated_world.friendlyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.friendlyTeam().getAllRobots().at(0).position().isClose(
//        Point(1, 0.3), 1e-6));
//    ASSERT_EQ(updated_world.enemyTeam().getAllRobots().size(), 1);
//    EXPECT_TRUE(updated_world.enemyTeam().getAllRobots().at(0).position().isClose(
//        Point(-0.5, -3), 1e-6));
//}
