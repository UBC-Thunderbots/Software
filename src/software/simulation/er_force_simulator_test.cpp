#include "software/simulation/er_force_simulator.h"

#include <gtest/gtest.h>
// TODO (#2419): remove this
#include <fenv.h>

#include "proto/message_translation/er_force_world.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2021_robot_constants.h"
#include "software/test_util/test_util.h"

class ErForceSimulatorTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        // TODO (#2419): remove this to re-enable sigfpe checks
        fedisableexcept(FE_INVALID | FE_OVERFLOW);
        auto realism_config = ErForceSimulator::createIdealRealismConfig();
        simulator = std::make_shared<ErForceSimulator>(TbotsProto::FieldType::DIV_B,
                                                       robot_constants, realism_config);
        simulator->resetCurrentTime();
    }

    std::shared_ptr<ErForceSimulator> simulator;
    RobotConstants_t robot_constants = create2021RobotConstants();
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
    RobotConstants_t robot_constants = create2021RobotConstants();
    auto realism_config              = ErForceSimulator::createIdealRealismConfig();
    std::shared_ptr<ErForceSimulator> simulator = std::make_shared<ErForceSimulator>(
        TbotsProto::FieldType::DIV_A, robot_constants, realism_config);
    simulator->resetCurrentTime();
    simulator->getField();

    EXPECT_EQ(simulator->getField(), Field::createSSLDivisionAField());
}

TEST(ErForceSimulatorFieldTest, check_field_B_configuration)
{
    RobotConstants_t robot_constants = create2021RobotConstants();
    auto realism_config              = ErForceSimulator::createIdealRealismConfig();
    std::shared_ptr<ErForceSimulator> simulator = std::make_shared<ErForceSimulator>(
        TbotsProto::FieldType::DIV_B, robot_constants, realism_config);
    simulator->resetCurrentTime();
    simulator->getField();

    EXPECT_EQ(simulator->getField(), Field::createSSLDivisionBField());
}
