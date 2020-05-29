#include "software/backend/robot_status.h"

#include <gtest/gtest.h>

TEST(RobotStatusTest, test_convert_robot_status_tbots_robot_msg)
{
    // Mock robot_status
    RobotStatus robot_status = {.robot = 1,
                                std::vector<std::string>(),
                                std::vector<std::string>(),
                                1.0,
                                1.0,
                                1.0,
                                1.0,
                                1,
                                true,
                                true,
                                .ball_in_beam = true,
                                true,
                                true,
                                .battery_voltage    = 4.0,
                                .capacitor_voltage  = 5.0,
                                .break_beam_reading = 2.0,
                                1.0,
                                .dribbler_temperature = 6.0,
                                .dribbler_speed       = 3,
                                .board_temperature    = 7.0,
                                1.0,
                                1,
                                true,
                                .fw_build_id = 2,
                                1};

    TbotsRobotMsg robot_msg = convertRobotStatusToTbotsRobotMsg(&robot_status);

    EXPECT_EQ(robot_msg.robot_id(), 1);
    EXPECT_EQ(robot_msg.break_beam_status().ball_in_beam(), true);
    EXPECT_EQ(robot_msg.break_beam_status().break_beam_reading(), 2.0);
    EXPECT_EQ(robot_msg.firmware_status().fw_build_id(), 2);
    EXPECT_EQ(robot_msg.dribbler_status().dribbler_rpm(), 3.0);
    EXPECT_EQ(robot_msg.power_status().battery_voltage(), 4.0);
    EXPECT_EQ(robot_msg.power_status().capacitor_voltage(), 5.0);
    EXPECT_EQ(robot_msg.temperature_status().dribbler_temperature(), 6.0);
    EXPECT_EQ(robot_msg.temperature_status().board_temperature(), 7.0);
}
