#include "software/simulation/simulator.h"

#include <gtest/gtest.h>

#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/primitive/primitive_msg_factory.h"
#include "software/test_util/test_util.h"

TEST(SimulatorTest, get_field)
{
    Field field = Field::createSSLDivisionBField();
    Simulator simulator(field);
    EXPECT_EQ(field, simulator.getField());
}

TEST(SimulatorTest, get_initial_timestamp)
{
    Simulator simulator(Field::createSSLDivisionBField());
    EXPECT_EQ(Timestamp::fromSeconds(0), simulator.getTimestamp());
}

TEST(SimulatorTest, timestamp_updates_with_simulation_steps)
{
    Simulator simulator(Field::createSSLDivisionBField());
    simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    EXPECT_EQ(Timestamp::fromSeconds(1.0 / 60.0), simulator.getTimestamp());
}

TEST(SimulatorTest, set_ball_state_when_ball_does_not_already_exist)
{
    Simulator simulator(Field::createSSLDivisionBField());

    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator.setBallState(ball_state);

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.balls_size());
    auto ball = detection_frame.balls(0);
    EXPECT_FLOAT_EQ(1000.0f, ball.x());
    EXPECT_FLOAT_EQ(2000.0f, ball.y());
}

TEST(SimulatorTest, set_ball_state_when_ball_already_exists)
{
    Simulator simulator(Field::createSSLDivisionBField());

    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator.setBallState(ball_state);

    BallState new_ball_state(Point(-3.5, 0.02), Vector(1, 1));
    simulator.setBallState(new_ball_state);

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.balls_size());
    auto ball = detection_frame.balls(0);
    EXPECT_FLOAT_EQ(-3500.0f, ball.x());
    EXPECT_FLOAT_EQ(20.0f, ball.y());
}

TEST(SimulatorTest, remove_ball_when_no_ball_exists)
{
    Simulator simulator(Field::createSSLDivisionBField());

    simulator.removeBall();

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(0, detection_frame.balls_size());
}

TEST(SimulatorTest, remove_ball_when_the_ball_already_exists)
{
    Simulator simulator(Field::createSSLDivisionBField());

    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator.setBallState(ball_state);
    simulator.removeBall();

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(0, detection_frame.balls_size());
}

TEST(SimualtorTest, add_zero_yellow_robots)
{
    Simulator simulator(Field::createSSLDivisionBField());

    simulator.addYellowRobots({});

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(0, detection_frame.robots_yellow_size());
}

TEST(SimulatorTest, add_multiple_yellow_robots_with_valid_ids)
{
    Simulator simulator(Field::createSSLDivisionBField());

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

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(3, detection_frame.robots_yellow_size());
}

TEST(SimulatorTest, add_yellow_robots_with_duplicate_ids)
{
    Simulator simulator(Field::createSSLDivisionBField());

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

TEST(SimulatorTest, add_yellow_robots_with_ids_that_already_exist_in_the_simulation)
{
    Simulator simulator(Field::createSSLDivisionBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(simulator.addYellowRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(simulator.addYellowRobots(states2), std::runtime_error);
}

TEST(SimualtorTest, add_zero_blue_robots)
{
    Simulator simulator(Field::createSSLDivisionBField());

    simulator.addBlueRobots({});

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(0, detection_frame.robots_blue_size());
}

TEST(SimulatorTest, add_multiple_blue_robots_with_valid_ids)
{
    Simulator simulator(Field::createSSLDivisionBField());

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

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    EXPECT_EQ(3, detection_frame.robots_blue_size());
}

TEST(SimulatorTest, add_blue_robots_with_duplicate_ids)
{
    Simulator simulator(Field::createSSLDivisionBField());

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

TEST(SimulatorTest, add_blue_robots_with_ids_that_already_exist_in_the_simulation)
{
    Simulator simulator(Field::createSSLDivisionBField());

    RobotState robot_state1(Point(1, 0), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::half());
    std::vector<RobotStateWithId> states1 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };

    EXPECT_NO_THROW(simulator.addBlueRobots(states1));

    RobotState robot_state2(Point(0, 0), Vector(3, 0), Angle::half(),
                            AngularVelocity::quarter());
    std::vector<RobotStateWithId> states2 = {
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    EXPECT_THROW(simulator.addBlueRobots(states2), std::runtime_error);
}

TEST(SimulatorTest, add_yellow_robot)
{
    Simulator simulator(Field::createSSLDivisionBField());

    auto wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(wrapper_packet->has_detection());
    EXPECT_EQ(0, wrapper_packet->detection().robots_yellow_size());

    simulator.addYellowRobot(Point(0, 1));

    wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(wrapper_packet->has_detection());
    EXPECT_EQ(1, wrapper_packet->detection().robots_yellow_size());

    auto robot = simulator.getRobotAtPosition(Point(0, 1));
    EXPECT_TRUE(robot.lock());
}

TEST(SimulatorTest, add_blue_robot)
{
    Simulator simulator(Field::createSSLDivisionBField());

    auto wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(wrapper_packet->has_detection());
    EXPECT_EQ(0, wrapper_packet->detection().robots_blue_size());

    simulator.addBlueRobot(Point(-0.5, -2));

    wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(wrapper_packet->has_detection());
    EXPECT_EQ(1, wrapper_packet->detection().robots_blue_size());

    auto robot = simulator.getRobotAtPosition(Point(-0.5, -2));
    EXPECT_TRUE(robot.lock());
}

TEST(SimulatorTest, simulation_step_updates_the_ball)
{
    // A sanity test to make sure stepping the simulation actually updates
    // the state of the world

    Simulator simulator(Field::createSSLDivisionBField());
    simulator.setBallState(BallState(Point(0.4, 0), Vector(-1.3, 2.01)));

    simulator.stepSimulation(Duration::fromSeconds(0.1));

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.balls_size());
    auto ball = detection_frame.balls(0);
    EXPECT_NEAR(270.0f, ball.x(), 10);
    EXPECT_NEAR(201.0f, ball.y(), 10);
}

TEST(SimulatorTest, simulate_yellow_robots_with_no_primitives)
{
    Simulator simulator(Field::createSSLDivisionBField());

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
    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_yellow_size());
    auto yellow_robot = detection_frame.robots_yellow(0);
    EXPECT_FLOAT_EQ(0.0f, yellow_robot.x());
    EXPECT_FLOAT_EQ(0.0f, yellow_robot.y());
}

TEST(SimulatorTest, simulate_single_yellow_robot_with_primitive)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(Field::createSSLDivisionBField());

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addYellowRobots(states);

    simulator.setYellowRobotPrimitive(
        1, createNanoPbPrimitive(
               *createMovePrimitive(Point(1, 0), 0.0, Angle::zero(), DribblerMode::OFF)));

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_yellow_size());
    auto yellow_robot = detection_frame.robots_yellow(0);
    EXPECT_NEAR(1000.0f, yellow_robot.x(), 200);
    EXPECT_NEAR(0.0f, yellow_robot.y(), 200);
}

TEST(SimulatorTest, simulate_blue_robots_with_no_primitives)
{
    Simulator simulator(Field::createSSLDivisionBField());

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
    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_blue_size());
    auto blue_robot = detection_frame.robots_blue(0);
    EXPECT_FLOAT_EQ(0.0f, blue_robot.x());
    EXPECT_FLOAT_EQ(0.0f, blue_robot.y());
}

TEST(SimulatorTest, simulate_single_blue_robot_with_primitive_defending_negative_side)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(Field::createSSLDivisionBField());

    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X);
    simulator.setBlueTeamDefendingSide(defending_side);

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addBlueRobots(states);

    simulator.setBlueRobotPrimitive(
        1, createNanoPbPrimitive(
               *createMovePrimitive(Point(1, 0), 0.0, Angle::zero(), DribblerMode::OFF)));

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_blue_size());
    auto blue_robot = detection_frame.robots_blue(0);
    EXPECT_NEAR(1000.0f, blue_robot.x(), 200);
    EXPECT_NEAR(0.0f, blue_robot.y(), 200);
}

TEST(SimulatorTest, simulate_single_blue_robot_with_primitive_defending_positive_side)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(Field::createSSLDivisionBField());

    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X);
    simulator.setBlueTeamDefendingSide(defending_side);

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addBlueRobots(states);

    simulator.setBlueRobotPrimitive(
        1, createNanoPbPrimitive(*createMovePrimitive(Point(1, -0.5), 0.0, Angle::zero(),
                                                      DribblerMode::OFF)));

    for (unsigned int i = 0; i < 240; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_blue_size());
    auto blue_robot = detection_frame.robots_blue(0);
    EXPECT_NEAR(-1000.0f, blue_robot.x(), 200);
    EXPECT_NEAR(500.0f, blue_robot.y(), 100);
    auto expected_robot_orientation = Angle::fromRadians(M_PI);
    auto actual_robot_orientation   = Angle::fromRadians(blue_robot.orientation());
    EXPECT_NEAR(
        0, expected_robot_orientation.minDiff(actual_robot_orientation).toDegrees(), 0.2);
}

TEST(SimulatorTest, simulate_single_yellow_robot_with_primitive_defending_negative_side)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(Field::createSSLDivisionBField());

    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X);
    simulator.setYellowTeamDefendingSide(defending_side);

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addYellowRobots(states);

    simulator.setYellowRobotPrimitive(
        1, createNanoPbPrimitive(
               *createMovePrimitive(Point(1, 0), 0.0, Angle::zero(), DribblerMode::OFF)));

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_yellow_size());
    auto yellow_robot = detection_frame.robots_yellow(0);
    EXPECT_NEAR(1000.0f, yellow_robot.x(), 200);
    EXPECT_NEAR(0.0f, yellow_robot.y(), 200);
}

TEST(SimulatorTest, simulate_single_yellow_robot_with_primitive_defending_positive_side)
{
    // Simulate a robot with a primitive to sanity check that everything is connected
    // properly and we can properly simulate robot firmware. We use the MovePrimitve
    // because it is very commonly used and so unlikely to be significantly changed
    // or removed, and its behaviour is easy to validate

    Simulator simulator(Field::createSSLDivisionBField());

    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X);
    simulator.setYellowTeamDefendingSide(defending_side);

    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 1, .robot_state = robot_state1},
    };
    simulator.addYellowRobots(states);

    simulator.setYellowRobotPrimitive(
        1, createNanoPbPrimitive(*createMovePrimitive(Point(1, -0.5), 0.0, Angle::zero(),
                                                      DribblerMode::OFF)));

    for (unsigned int i = 0; i < 240; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(1, detection_frame.robots_yellow_size());
    auto yellow_robot = detection_frame.robots_yellow(0);
    EXPECT_NEAR(-1000.0f, yellow_robot.x(), 200);
    EXPECT_NEAR(500.0f, yellow_robot.y(), 100);
    auto expected_robot_orientation = Angle::fromRadians(M_PI);
    auto actual_robot_orientation   = Angle::fromRadians(yellow_robot.orientation());
    EXPECT_NEAR(
        0, expected_robot_orientation.minDiff(actual_robot_orientation).toDegrees(), 0.2);
}

TEST(SimulatorTest, simulate_multiple_blue_and_yellow_robots_with_primitives)
{
    // Simulate multiple robots with primitives to sanity check that everything is
    // connected properly and we can properly simulate multiple instances of the robot
    // firmware at once. We use the MovePrimitve because it is very commonly used and so
    // unlikely to be significantly changed or removed, and its behaviour is easy to
    // validate

    Simulator simulator(Field::createSSLDivisionBField());

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

    simulator.setBlueRobotPrimitive(
        1, createNanoPbPrimitive(*createMovePrimitive(Point(-1, -1), 0.0, Angle::zero(),
                                                      DribblerMode::OFF)));
    simulator.setBlueRobotPrimitive(
        2, createNanoPbPrimitive(*createMovePrimitive(Point(-3, 0), 0.0, Angle::half(),
                                                      DribblerMode::OFF)));

    simulator.setYellowRobotPrimitive(
        1, createNanoPbPrimitive(
               *createMovePrimitive(Point(1, 1), 0.0, Angle::zero(), DribblerMode::OFF)));
    simulator.setYellowRobotPrimitive(
        2, createNanoPbPrimitive(*createMovePrimitive(Point(3, -2), 0.0, Angle::zero(),
                                                      DribblerMode::OFF)));

    for (unsigned int i = 0; i < 120; i++)
    {
        simulator.stepSimulation(Duration::fromSeconds(1.0 / 60.0));
    }

    // TODO: These tests are currently very lenient, and don't test final velocities.
    //  This is because they currently rely on controller dynamics, and the existing
    //  bang-bang controller tends to overshoot with the current physics damping
    //  constants. In order to help decouple these tests from the controller / damping,
    //  the test tolerances are larger for now. They should be tightened again when the
    //  new controller is implemented.
    //  https://github.com/UBC-Thunderbots/Software/issues/1187

    auto ssl_wrapper_packet = simulator.getSSLWrapperPacket();
    ASSERT_TRUE(ssl_wrapper_packet);
    ASSERT_TRUE(ssl_wrapper_packet->has_detection());
    auto detection_frame = ssl_wrapper_packet->detection();
    ASSERT_EQ(2, detection_frame.robots_yellow_size());
    ASSERT_EQ(2, detection_frame.robots_blue_size());

    auto yellow_robots  = detection_frame.robots_yellow();
    auto yellow_robot_1 = std::find_if(
        yellow_robots.begin(), yellow_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 1; });
    ASSERT_NE(yellow_robot_1, yellow_robots.end());
    EXPECT_NEAR(1000.0f, yellow_robot_1->x(), 200);
    EXPECT_NEAR(1000.0f, yellow_robot_1->y(), 200);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(yellow_robot_1->orientation()),
        Angle::fromDegrees(10)));

    auto yellow_robot_2 = std::find_if(
        yellow_robots.begin(), yellow_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 2; });
    ASSERT_NE(yellow_robot_2, yellow_robots.end());
    EXPECT_NEAR(3000.0f, yellow_robot_2->x(), 200);
    EXPECT_NEAR(-2000.0f, yellow_robot_2->y(), 200);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(yellow_robot_2->orientation()),
        Angle::fromDegrees(10)));

    auto blue_robots  = detection_frame.robots_blue();
    auto blue_robot_1 = std::find_if(
        blue_robots.begin(), blue_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 1; });
    ASSERT_NE(blue_robot_1, blue_robots.end());
    EXPECT_NEAR(-1000.0f, blue_robot_1->x(), 300);
    EXPECT_NEAR(-1000.0f, blue_robot_1->y(), 300);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::zero(), Angle::fromRadians(blue_robot_1->orientation()),
        Angle::fromDegrees(10)));

    auto blue_robot_2 = std::find_if(
        blue_robots.begin(), blue_robots.end(),
        [](SSLProto::SSL_DetectionRobot robot) { return robot.robot_id() == 2; });
    ASSERT_NE(blue_robot_2, blue_robots.end());
    EXPECT_NEAR(-3000.0f, blue_robot_2->x(), 300);
    EXPECT_NEAR(0.0f, blue_robot_2->y(), 300);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::half(), Angle::fromRadians(blue_robot_2->orientation()),
        Angle::fromDegrees(10)));
}
