#include "software/simulation/er_force_simulator.h"

#include <gtest/gtest.h>

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2021_robot_constants.h"
#include "software/test_util/test_util.h"

class ErForceSimulatorTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        simulator_config = std::make_shared<const SimulatorConfig>();
        simulator = std::make_shared<ErForceSimulator>(Field::createSSLDivisionBField(),
                                                       robot_constants, wheel_constants,
                                                       simulator_config);
        simulator->resetCurrentTime();
    }

    std::shared_ptr<ErForceSimulator> simulator;
    std::shared_ptr<const SimulatorConfig> simulator_config;
    RobotConstants_t robot_constants = create2021RobotConstants();
    WheelConstants wheel_constants   = create2021WheelConstants();
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

TEST_F(ErForceSimulatorTest, position_robots_for_default_kickoff)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -2.5)});
    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
        {Point(3, 2.5), Point(3, 1.5), Point(3, 0.5), Point(3, -0.5), Point(3, -1.5),
         Point(3, -2.5)});

    simulator->addYellowRobots(friendly_robots);
    simulator->addBlueRobots(enemy_robots);
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
    simulator->addYellowRobots(friendly_robots);
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
            // Simulator position seems flipped from visualizer, temporary for now
            double x_difference = fabs(y - friendly_robots[i].robot_state.position().x());
            double y_difference =
                fabs(-x - friendly_robots[i].robot_state.position().y());
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
