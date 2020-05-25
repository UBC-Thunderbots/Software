#include "software/backend/radio_backend.h"

#include <gtest/gtest.h>

TEST(RadioBackendTest, test_convert_robot_status_tbots_robot_msg)
{
    // Create RobotStatus
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
                                .battery_voltage    = 1.0,
                                .capacitor_voltage  = 1.0,
                                .break_beam_reading = 1.0,
                                1.0,
                                .dribbler_temperature = 1.0,
                                .dribbler_speed       = 1,
                                .board_temperature    = 1.0,
                                1.0,
                                1,
                                true,
                                .fw_build_id = 1,
                                1};

    // TODO write unit test and clean up struct
    // RadioBackend radio_backend
    // TbotsRobotMsg robot_msg = radio_backend.convert...();

    EXPECT_DOUBLE_EQ(0.6, 0.6);
}
