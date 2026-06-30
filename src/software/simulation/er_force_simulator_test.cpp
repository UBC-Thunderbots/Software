#include "software/simulation/er_force_simulator.h"

#include <gtest/gtest.h>

#include "proto/message_translation/er_force_world.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/robot_constants.h"
#include "software/geom/vector.h"
#include "software/physics/euclidean_to_wheel.h"
#include "software/test_util/test_util.h"

class ErForceSimulatorTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        auto realism_config = ErForceSimulator::createDefaultRealismConfig();
        simulator = std::make_shared<ErForceSimulator>(TbotsProto::FieldType::DIV_B,
                                                       robot_constants, realism_config);
        simulator->resetCurrentTime();
    }

    std::shared_ptr<ErForceSimulator> simulator;
    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
};

TEST_F(ErForceSimulatorTest, set_ball_state_when_ball_does_not_already_exist)
{
    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator->setBallState(ball_state);
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    auto ssl_wrapper_packets                = simulator->getSSLWrapperPackets();
    bool at_least_one_wrapper_packet_passes = false;
    for (const auto& ssl_wrapper_packet : ssl_wrapper_packets)
    {
        if (ssl_wrapper_packet.has_detection())
        {
            auto detection_frame = ssl_wrapper_packet.detection();
            for (const auto& ball : detection_frame.balls())
            {
                if (ball.has_x() && ball.has_y())
                {
                    double x_difference = fabs(ball.x() - 1000.0f);
                    double y_difference = fabs(ball.y() - 2000.0f);
                    if (x_difference < 1.0f && y_difference < 1.0f)
                    {
                        at_least_one_wrapper_packet_passes = true;
                    }
                }
            }
        }
    }
    EXPECT_TRUE(at_least_one_wrapper_packet_passes);
}

TEST_F(ErForceSimulatorTest, set_ball_state_when_ball_already_exists)
{
    BallState ball_state(Point(1, 2), Vector(0, -3));
    simulator->setBallState(ball_state);
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    BallState new_ball_state(Point(-3.5, 0.02), Vector(1, 1));
    simulator->setBallState(new_ball_state);
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    auto ssl_wrapper_packets                = simulator->getSSLWrapperPackets();
    bool at_least_one_wrapper_packet_passes = false;
    for (const auto& ssl_wrapper_packet : ssl_wrapper_packets)
    {
        if (ssl_wrapper_packet.has_detection())
        {
            auto detection_frame = ssl_wrapper_packet.detection();
            for (const auto& ball : detection_frame.balls())
            {
                if (ball.has_x() && ball.has_y())
                {
                    double x_difference = fabs(ball.x() - (-3500.0f));
                    double y_difference = fabs(ball.y() - 20.0f);
                    if (x_difference < 2.0f && y_difference < 2.0f)
                    {
                        at_least_one_wrapper_packet_passes = true;
                    }
                }
            }
        }
    }
    EXPECT_TRUE(at_least_one_wrapper_packet_passes);
}

TEST_F(ErForceSimulatorTest, position_robots_for_default_kickoff)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -2.5)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(3, 2.5), Point(3, 1.5), Point(3, 0.5), Point(3, -0.5), Point(3, -1.5),
         Point(3, -2.5)});


    simulator->setYellowRobots(friendly_robots);
    simulator->setBlueRobots(enemy_robots);


    simulator->stepSimulation(Duration::fromMilliseconds(10));

    auto ssl_wrapper_packets = simulator->getSSLWrapperPackets();
    bool blue_visible        = false;
    bool yellow_visible      = false;


    for (const auto& ssl_wrapper_packet : ssl_wrapper_packets)
    {
        if (ssl_wrapper_packet.has_detection())
        {
            auto detection_frame = ssl_wrapper_packet.detection();
            if (detection_frame.robots_yellow_size() == 6)
            {
                yellow_visible = true;
            }

            if (detection_frame.robots_blue_size() == 6)
            {
                blue_visible = true;
            }
        }
    }

    EXPECT_TRUE(blue_visible && yellow_visible);
}

TEST_F(ErForceSimulatorTest, add_yellow_robots)
{
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(0, 2), Point(1, 3), Point(2, 4)});
    simulator->setYellowRobots(friendly_robots);
    simulator->stepSimulation(Duration::fromMilliseconds(10));


    auto ssl_wrapper_packets = simulator->getSSLWrapperPackets();
    bool yellow_visible      = false;

    for (const auto& ssl_wrapper_packet : ssl_wrapper_packets)
    {
        if (ssl_wrapper_packet.has_detection())
        {
            auto detection_frame = ssl_wrapper_packet.detection();
            if (detection_frame.robots_yellow_size() == 3)
            {
                yellow_visible = true;
            }
        }
    }

    auto simState      = simulator->getSimulatorState();
    auto yellow_robots = simState.yellow_robots();

    for (const auto& sim_robot : yellow_robots)
    {
        auto x = sim_robot.p_x();
        auto y = sim_robot.p_y();

        for (unsigned int i = 0; i < friendly_robots.size(); i++)
        {
            double x_difference = fabs(x - friendly_robots[i].robot_state.position().x());
            double y_difference = fabs(y - friendly_robots[i].robot_state.position().y());
            if (x_difference < 0.2f && y_difference < 0.2f)
            {
                friendly_robots.erase(friendly_robots.begin() + i);
                break;
            }
        }
    }

    EXPECT_EQ(0, friendly_robots.size());
    EXPECT_EQ(3, yellow_robots.size());
    EXPECT_TRUE(yellow_visible);
}

TEST_F(ErForceSimulatorTest, yellow_robot_velocity_test)
{
    RobotState robot_state1(Point(1, 0), Vector(2, 0), Angle::zero(),
                            AngularVelocity::zero());
    RobotState robot_state2(Point(0, 1), Vector(0, 2), Angle::zero(),
                            AngularVelocity::zero());

    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
    };

    simulator->setYellowRobots(states);
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    auto simState      = simulator->getSimulatorState();
    auto yellow_robots = simState.yellow_robots();

    EXPECT_TRUE(TestUtil::equalWithinTolerance(yellow_robots[0].v_x(), 2.0, 0.1));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(yellow_robots[0].v_y(), 0, 0.1));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(yellow_robots[1].v_x(), 0, 0.1));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(yellow_robots[1].v_y(), 2, 0.1));
}

TEST_F(ErForceSimulatorTest, yellow_robot_orientation_test)
{
    RobotState robot_state1(Point(0, 0), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    RobotState robot_state2(Point(0, 1), Vector(0, 0), Angle::quarter(),
                            AngularVelocity::quarter());
    RobotState robot_state3(Point(0, 2), Vector(0, 0), Angle::half(),
                            AngularVelocity::half());

    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
        RobotStateWithId{.id = 2, .robot_state = robot_state3},
    };

    simulator->setYellowRobots(states);
    simulator->stepSimulation(Duration::fromSeconds(10));

    auto simState      = simulator->getSimulatorState();
    auto yellow_robots = simState.yellow_robots();

    auto robot_zero    = createRobot(yellow_robots[0], Timestamp::fromSeconds(10));
    auto robot_quarter = createRobot(yellow_robots[1], Timestamp::fromSeconds(10));
    auto robot_half    = createRobot(yellow_robots[2], Timestamp::fromSeconds(10));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_zero.orientation(), Angle::zero(),
                                               Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_quarter.orientation(),
                                               Angle::quarter(), Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_half.orientation(), Angle::half(),
                                               Angle::fromDegrees(1)));
}

TEST_F(ErForceSimulatorTest, yellow_robot_add_robots_and_change_position)
{
    RobotState robot_state1(Point(1, -1), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    RobotState robot_state2(Point(2, -2), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());
    RobotState robot_state3(Point(3, -3), Vector(0, 0), Angle::zero(),
                            AngularVelocity::zero());

    std::vector<RobotStateWithId> states = {
        RobotStateWithId{.id = 0, .robot_state = robot_state1},
        RobotStateWithId{.id = 1, .robot_state = robot_state2},
        RobotStateWithId{.id = 2, .robot_state = robot_state3},
    };

    simulator->setYellowRobots(states);
    simulator->stepSimulation(Duration::fromMilliseconds(10));

    auto simState      = simulator->getSimulatorState();
    auto yellow_robots = simState.yellow_robots();

    auto robot_1 = createRobot(yellow_robots[0], Timestamp::fromSeconds(10));
    auto robot_2 = createRobot(yellow_robots[1], Timestamp::fromSeconds(10));
    auto robot_3 = createRobot(yellow_robots[2], Timestamp::fromSeconds(10));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_1.currentState(), robot_state1, 0.1,
                                               Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_2.currentState(), robot_state2, 0.1,
                                               Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_3.currentState(), robot_state3, 0.1,
                                               Angle::fromDegrees(1)));

    EXPECT_EQ(states.size(), yellow_robots.size());

    RobotState new_robot_state1(Point(4, 0), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero());
    RobotState new_robot_state2(Point(2, -2), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero());
    RobotState new_robot_state3(Point(-2, -1), Vector(0, 0), Angle::zero(),
                                AngularVelocity::zero());

    std::vector<RobotStateWithId> new_states = {
        RobotStateWithId{.id = 0, .robot_state = new_robot_state1},
        RobotStateWithId{.id = 1, .robot_state = new_robot_state2},
        RobotStateWithId{.id = 2, .robot_state = new_robot_state3},
    };

    simulator->setYellowRobots(new_states);
    simulator->stepSimulation(Duration::fromMilliseconds(10));

    simState      = simulator->getSimulatorState();
    yellow_robots = simState.yellow_robots();

    robot_1 = createRobot(yellow_robots[0], Timestamp::fromSeconds(20));
    robot_2 = createRobot(yellow_robots[1], Timestamp::fromSeconds(20));
    robot_3 = createRobot(yellow_robots[2], Timestamp::fromSeconds(20));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_1.currentState(), new_robot_state1,
                                               0.1, Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_2.currentState(), new_robot_state2,
                                               0.1, Angle::fromDegrees(1)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot_3.currentState(), new_robot_state3,
                                               0.1, Angle::fromDegrees(1)));

    EXPECT_EQ(new_states.size(), yellow_robots.size());
}


TEST(ErForceSimulatorFieldTest, check_field_A_configuration)
{
    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
    auto realism_config = ErForceSimulator::createDefaultRealismConfig();
    std::shared_ptr<ErForceSimulator> simulator = std::make_shared<ErForceSimulator>(
        TbotsProto::FieldType::DIV_A, robot_constants, realism_config);
    simulator->resetCurrentTime();
    simulator->getField();

    EXPECT_EQ(simulator->getField(), Field::createSSLDivisionAField());
}

TEST(ErForceSimulatorFieldTest, check_field_B_configuration)
{
    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
    auto realism_config = ErForceSimulator::createDefaultRealismConfig();
    std::shared_ptr<ErForceSimulator> simulator = std::make_shared<ErForceSimulator>(
        TbotsProto::FieldType::DIV_B, robot_constants, realism_config);
    simulator->resetCurrentTime();
    simulator->getField();

    EXPECT_EQ(simulator->getField(), Field::createSSLDivisionBField());
}

class ErForceSimulatorRampingTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        auto realism_config = ErForceSimulator::createDefaultRealismConfig();
        simulator = std::make_shared<ErForceSimulator>(TbotsProto::FieldType::DIV_B,
                                                       robot_constants, realism_config,
                                                       /*ramping=*/true);
    }

    // Forwarding wrapper so TEST_F bodies (which derive from this fixture) can reach the
    // private method through this friend class.
    std::unique_ptr<TbotsProto::DirectControlPrimitive> rampVelocityPrimitive(
        const Vector& current_local_velocity,
        const AngularVelocity& current_local_angular_velocity,
        TbotsProto::DirectControlPrimitive& target_velocity_primitive,
        Duration time_to_ramp)
    {
        return simulator->getRampedVelocityPrimitive(
            current_local_velocity, current_local_angular_velocity,
            target_velocity_primitive, time_to_ramp);
    }

    // Builds a direct-velocity-control primitive with the given local target velocity.
    static TbotsProto::DirectControlPrimitive makeTargetPrimitive(
        const Vector& velocity, const AngularVelocity& angular_velocity)
    {
        return createDirectControlPrimitive(velocity, angular_velocity,
                                            /*dribbler_rpm=*/0,
                                            TbotsProto::AutoChipOrKick())
            ->direct_control();
    }

    std::shared_ptr<ErForceSimulator> simulator;
    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
};

TEST_F(ErForceSimulatorRampingTest, passes_target_through_when_within_acceleration_limit)
{
    const Vector target_velocity(0.5, -0.3);
    const AngularVelocity target_angular = AngularVelocity::fromRadians(0.2);

    auto target_primitive = makeTargetPrimitive(target_velocity, target_angular);

    // Start from rest, but allow a large ramp window so nothing clips.
    auto ramped = rampVelocityPrimitive(Vector(0, 0), AngularVelocity::zero(),
                                        target_primitive, Duration::fromSeconds(10.0));

    const auto& velocity = ramped->motor_control().direct_velocity_control().velocity();
    EXPECT_NEAR(velocity.x_component_meters(), target_velocity.x(), 1e-9);
    EXPECT_NEAR(velocity.y_component_meters(), target_velocity.y(), 1e-9);
    EXPECT_NEAR(ramped->motor_control()
                    .direct_velocity_control()
                    .angular_velocity()
                    .radians_per_second(),
                target_angular.toRadians(), 1e-9);
}

TEST_F(ErForceSimulatorRampingTest, holds_velocity_when_already_at_target)
{
    const Vector velocity(1.0, 0.5);
    const AngularVelocity angular = AngularVelocity::fromRadians(0.4);

    auto target_primitive = makeTargetPrimitive(velocity, angular);

    auto ramped = rampVelocityPrimitive(velocity, angular, target_primitive,
                                        Duration::fromSeconds(0.001));

    const auto& out = ramped->motor_control().direct_velocity_control().velocity();
    EXPECT_NEAR(out.x_component_meters(), velocity.x(), 1e-9);
    EXPECT_NEAR(out.y_component_meters(), velocity.y(), 1e-9);
}

TEST_F(ErForceSimulatorRampingTest, ramps_in_motor_service_frame_when_clipping)
{
    const Vector current_velocity(0.0, 0.0);
    const AngularVelocity current_angular = AngularVelocity::zero();
    const Vector target_velocity(3.0, 0.5);
    const AngularVelocity target_angular = AngularVelocity::fromRadians(1.0);
    // Tiny timestep forces the acceleration limit to clip hard.
    const Duration time_to_ramp = Duration::fromSeconds(0.01);

    EuclideanToWheel euclidean_to_wheel(robot_constants);

    EuclideanSpace_t current_euclidean{current_velocity.x(), current_velocity.y(),
                                       current_angular.toRadians()};
    EuclideanSpace_t target_euclidean{target_velocity.x(), target_velocity.y(),
                                      target_angular.toRadians()};
    WheelSpace_t ramped_wheel = euclidean_to_wheel.rampWheelVelocity(
        euclidean_to_wheel.getWheelVelocity(current_euclidean),
        euclidean_to_wheel.getWheelVelocity(target_euclidean), time_to_ramp.toSeconds());
    EuclideanSpace_t expected = euclidean_to_wheel.getEuclideanVelocity(ramped_wheel);

    ASSERT_LT(expected[0], target_velocity.x());

    auto target_primitive = makeTargetPrimitive(target_velocity, target_angular);
    auto ramped           = rampVelocityPrimitive(current_velocity, current_angular,
                                                  target_primitive, time_to_ramp);

    const auto& velocity = ramped->motor_control().direct_velocity_control().velocity();
    EXPECT_NEAR(velocity.x_component_meters(), expected[0], 1e-9);
    EXPECT_NEAR(velocity.y_component_meters(), expected[1], 1e-9);
    EXPECT_NEAR(ramped->motor_control()
                    .direct_velocity_control()
                    .angular_velocity()
                    .radians_per_second(),
                expected[2], 1e-9);

    EuclideanSpace_t rotated_current{-current_velocity.y(), current_velocity.x(),
                                     current_angular.toRadians()};
    EuclideanSpace_t rotated_target{-target_velocity.y(), target_velocity.x(),
                                    target_angular.toRadians()};
    EuclideanSpace_t rotated_ramped =
        euclidean_to_wheel.getEuclideanVelocity(euclidean_to_wheel.rampWheelVelocity(
            euclidean_to_wheel.getWheelVelocity(rotated_current),
            euclidean_to_wheel.getWheelVelocity(rotated_target),
            time_to_ramp.toSeconds()));

    EXPECT_GT(std::abs(rotated_ramped[1] - expected[0]), 1e-3);
}
