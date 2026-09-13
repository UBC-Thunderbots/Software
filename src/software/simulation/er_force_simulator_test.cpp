#include "software/simulation/er_force_simulator.h"

#include <gtest/gtest.h>

#include "proto/message_translation/er_force_world.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/constants.h"
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

TEST_F(ErForceSimulatorTest, robots_duelling_over_the_ball_stay_upright)
{
    // Two robots drive into the ball from opposite sides with their dribblers running.
    // The perfect dribbler holds the ball with a constraint between the robot and the
    // ball, which used to tip the robots over when two of them pulled on the same ball.
    // A tipped over robot is considered flipped and gets teleported to the side of the
    // field by Simulator::resetFlipped.
    constexpr double DISTANCE_FROM_BALL_METERS     = 0.11;
    constexpr double DRIVE_SPEED_METERS_PER_SECOND = 1.0;

    simulator->setBallState(BallState(Point(0, 0), Vector(0, 0)));
    simulator->setYellowRobots({RobotStateWithId{
        .id          = 0,
        .robot_state = RobotState(Point(DISTANCE_FROM_BALL_METERS, 0), Vector(0, 0),
                                  Angle::half(), AngularVelocity::zero())}});
    simulator->setBlueRobots({RobotStateWithId{
        .id          = 0,
        .robot_state = RobotState(Point(-DISTANCE_FROM_BALL_METERS, 0), Vector(0, 0),
                                  Angle::zero(), AngularVelocity::zero())}});

    // Both robots drive forwards, towards each other and the ball, while dribbling
    TbotsProto::PrimitiveSet primitive_set;
    (*primitive_set.mutable_robot_primitives())[0] = *createDirectControlPrimitive(
        Vector(DRIVE_SPEED_METERS_PER_SECOND, 0), AngularVelocity::zero(),
        robot_constants.indefinite_dribbler_speed_rpm, TbotsProto::AutoChipOrKick());

    for (unsigned int step = 0; step < 400; step++)
    {
        simulator->setYellowRobotPrimitiveSet(primitive_set,
                                              std::make_unique<TbotsProto::World>());
        simulator->setBlueRobotPrimitiveSet(primitive_set,
                                            std::make_unique<TbotsProto::World>());
        simulator->stepSimulation(Duration::fromMilliseconds(5));
    }

    auto sim_state = simulator->getSimulatorState();
    ASSERT_EQ(1, sim_state.yellow_robots_size());
    ASSERT_EQ(1, sim_state.blue_robots_size());

    for (const auto& robot : {sim_state.yellow_robots(0), sim_state.blue_robots(0)})
    {
        // The z component of the robot's local z axis in world coordinates. It is 1 when
        // the robot stands flat on the field and decreases as the robot tips over.
        const double i                 = robot.rotation().i();
        const double j                 = robot.rotation().j();
        const double upright_component = 1.0 - 2.0 * (i * i + j * j);

        EXPECT_GT(upright_component, std::cos(Angle::fromDegrees(10).toRadians()))
            << "Robot tipped over while duelling for the ball";

        // A robot that stays upright also stays within the width of the field, rather
        // than being teleported to the side by resetFlipped
        EXPECT_LT(std::abs(robot.p_y()), simulator->getField().yLength() / 2);
    }
}

TEST_F(ErForceSimulatorTest, simulator_state_rotation_matches_robot_orientation)
{
    const Angle orientation = Angle::fromRadians(0.7);

    simulator->setYellowRobots({RobotStateWithId{
        .id          = 0,
        .robot_state = RobotState(Point(0, 0), Vector(0, 0), orientation,
                                  AngularVelocity::zero())}});
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    auto sim_state = simulator->getSimulatorState();
    ASSERT_EQ(1, sim_state.yellow_robots_size());
    const auto& rotation = sim_state.yellow_robots(0).rotation();

    // A robot standing on the field is only rotated about the z axis, so the rotation
    // quaternion is (i, j, k, real) = (0, 0, sin(angle / 2), cos(angle / 2))
    EXPECT_NEAR(rotation.i(), 0.0, 1e-3);
    EXPECT_NEAR(rotation.j(), 0.0, 1e-3);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Angle::fromRadians(2 * std::atan2(rotation.k(), rotation.real())), orientation,
        Angle::fromDegrees(1)));
}

class ErForceSimulatorRealismTest : public ::testing::Test
{
   protected:
    // Creates a simulator whose realism config is the default one, with the given
    // modification applied
    void createSimulator(
        const std::function<void(RealismConfigErForce&)>& configure_realism)
    {
        auto realism_config = ErForceSimulator::createDefaultRealismConfig();
        configure_realism(*realism_config);
        simulator = std::make_shared<ErForceSimulator>(TbotsProto::FieldType::DIV_B,
                                                       robot_constants, realism_config);
        simulator->resetCurrentTime();
    }

    // Adds a single stationary yellow robot at the center of the field
    void addYellowRobot()
    {
        simulator->setYellowRobots({RobotStateWithId{
            .id          = 0,
            .robot_state = RobotState(Point(0, 0), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero())}});
    }

    // Returns all yellow robot detections across all cameras of the latest packets
    std::vector<SSLProto::SSL_DetectionRobot> getYellowDetections()
    {
        std::vector<SSLProto::SSL_DetectionRobot> detections;
        for (const auto& packet : simulator->getSSLWrapperPackets())
        {
            for (const auto& robot : packet.detection().robots_yellow())
            {
                detections.push_back(robot);
            }
        }
        return detections;
    }

    std::shared_ptr<ErForceSimulator> simulator;
    robot_constants::RobotConstants robot_constants =
        robot_constants::createRobotConstants();
};

TEST_F(ErForceSimulatorRealismTest, robots_are_always_detected_by_default)
{
    createSimulator([](RealismConfigErForce&) {});
    addYellowRobot();
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    EXPECT_FALSE(getYellowDetections().empty());
}

TEST_F(ErForceSimulatorRealismTest, robots_are_never_detected_when_always_missing)
{
    createSimulator([](RealismConfigErForce& realism)
                    { realism.set_missing_robot_detections(1.0f); });
    addYellowRobot();
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    EXPECT_TRUE(getYellowDetections().empty());
}

TEST_F(ErForceSimulatorRealismTest, rotated_robot_detections_are_reported_on_top)
{
    createSimulator(
        [](RealismConfigErForce& realism)
        {
            realism.set_rotated_robot_detections_start(1.0f);
            realism.set_rotated_robot_detections_stop(0.0f);
        });
    addYellowRobot();
    simulator->stepSimulation(Duration::fromMilliseconds(5));

    auto detections = getYellowDetections();
    ASSERT_EQ(2, detections.size());

    // The robot is reported once with its own id and once with the id of the pattern
    // that its own pattern turns into when rotated by 90 degrees
    EXPECT_EQ(0, detections[0].robot_id());
    EXPECT_NE(detections[0].robot_id(), detections[1].robot_id());
    EXPECT_NEAR(detections[1].x(), detections[0].x(), 1e-3);
    EXPECT_NEAR(detections[1].y(), detections[0].y(), 1e-3);
    EXPECT_NEAR(detections[1].orientation(), detections[0].orientation() + M_PI_2, 1e-3);
}

TEST_F(ErForceSimulatorRealismTest, commands_are_only_applied_after_the_command_delay)
{
    constexpr double COMMAND_DELAY_SECONDS         = 0.1;
    constexpr double DRIVE_SPEED_METERS_PER_SECOND = 1.0;

    TbotsProto::PrimitiveSet primitive_set;
    (*primitive_set.mutable_robot_primitives())[0] = *createDirectControlPrimitive(
        Vector(DRIVE_SPEED_METERS_PER_SECOND, 0), AngularVelocity::zero(),
        /*dribbler_rpm=*/0, TbotsProto::AutoChipOrKick());

    // Drives the robot forwards for the given duration and returns its forward velocity
    const auto driveFor = [&](const Duration& duration)
    {
        for (unsigned int step = 0; step * 5 < duration.toMilliseconds(); step++)
        {
            simulator->setYellowRobotPrimitiveSet(primitive_set,
                                                  std::make_unique<TbotsProto::World>());
            simulator->stepSimulation(Duration::fromMilliseconds(5));
        }
        return simulator->getSimulatorState().yellow_robots(0).v_x();
    };

    // Without a command delay the robot starts driving right away
    createSimulator([](RealismConfigErForce&) {});
    addYellowRobot();
    const double velocity_without_delay =
        driveFor(Duration::fromSeconds(COMMAND_DELAY_SECONDS / 2));
    EXPECT_GT(velocity_without_delay, 0.05);

    // With a command delay the robot has not received anything yet at the same point in
    // time, so it only drifts by the tiny amount it takes to settle onto the field
    createSimulator(
        [](RealismConfigErForce& realism)
        {
            realism.set_command_delay(
                static_cast<int64_t>(COMMAND_DELAY_SECONDS * NANOSECONDS_PER_SECOND));
        });
    addYellowRobot();
    EXPECT_NEAR(driveFor(Duration::fromSeconds(COMMAND_DELAY_SECONDS / 2)), 0.0, 0.01);

    // Once the delay has passed, the robot drives just like it does without a delay
    EXPECT_GT(driveFor(Duration::fromSeconds(1.0)), 0.1);
}

TEST_F(ErForceSimulatorTest, corner_blocks_keep_the_ball_out_of_the_field_corners)
{
    // The corners of the field are blocked off by triangular blocks, so a ball rolling
    // into a corner is deflected by them instead of coming to rest in the corner itself
    constexpr double CORNER_BLOCK_CATHETUS_METERS = 0.09;

    const double corner_x =
        simulator->getField().xLength() / 2 + simulator->getField().boundaryMargin();
    const double corner_y =
        simulator->getField().yLength() / 2 + simulator->getField().boundaryMargin();

    simulator->setBallState(BallState(Point(3.9, 2.4), Vector(2.5, 2.5)));

    // How far the ball gets into the corner, measured as the distance from the corner
    // along both axes summed up. The block face runs diagonally across the corner, so
    // this value cannot get below the length of its cathetus while the block is there.
    double closest_approach_to_corner = std::numeric_limits<double>::max();
    for (unsigned int step = 0; step < 400; step++)
    {
        simulator->stepSimulation(Duration::fromMilliseconds(5));

        for (const auto& packet : simulator->getSSLWrapperPackets())
        {
            for (const auto& ball : packet.detection().balls())
            {
                const double x = std::abs(ball.x() * METERS_PER_MILLIMETER);
                const double y = std::abs(ball.y() * METERS_PER_MILLIMETER);
                closest_approach_to_corner =
                    std::min(closest_approach_to_corner, (corner_x - x) + (corner_y - y));
            }
        }
    }

    // Without the corner block the ball rolls right up into the corner, where it only
    // keeps its own radius of distance from each of the two walls
    EXPECT_GT(closest_approach_to_corner, CORNER_BLOCK_CATHETUS_METERS * 0.75);
}
