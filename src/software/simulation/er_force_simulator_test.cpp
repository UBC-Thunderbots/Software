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
        simulator = std::make_shared<ErForceSimulator>(FieldType::DIV_B,
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
