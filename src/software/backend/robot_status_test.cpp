#include "software/backend/robot_status.h"

#include <gtest/gtest.h>

#include "shared/proto/tbots_robot_msg.pb.h"

TEST(RobotStatusTest, test_convert_robot_status_tbots_robot_msg_valid_robot_msg)
{
    // Mock robot_messages with valid ErrorCode messages
    std::vector<std::string> robot_messages = {
        "charge timeout",
        "wheel 0 motor hot",
        "wheel 1 motor hot",
        "wheel 2 motor hot",
        "wheel 3 motor hot",
        "dribbler motor hot",
        "wheel 0 encoder not commutating",
        "wheel 1 encoder not commutating",
        "wheel 2 encoder not commutating",
        "wheel 3 encoder not commutating",
        "wheel 0 Hall sensor stuck low",
        "wheel 1 Hall sensor stuck low",
        "wheel 2 Hall sensor stuck low",
        "wheel 3 Hall sensor stuck low",
        "dribbler Hall sensor stuck low",
        "wheel 0 Hall sensor stuck high",
        "wheel 1 Hall sensor stuck high",
        "wheel 2 Hall sensor stuck high",
        "wheel 3 Hall sensor stuck high",
        "dribbler Hall sensor stuck high",
    };

    // Mock expected return ErrorCode enums from robot_messages
    std::vector<ErrorCode> expected_error_code = {CHARGE_TIMEOUT,
                                                  WHEEL_0_MOTOR_HOT,
                                                  WHEEL_1_MOTOR_HOT,
                                                  WHEEL_2_MOTOR_HOT,
                                                  WHEEL_3_MOTOR_HOT,
                                                  DRIBBLER_MOTOR_HOT,
                                                  WHEEL_0_ENCODER_NOT_COMMUTATING,
                                                  WHEEL_1_ENCODER_NOT_COMMUTATING,
                                                  WHEEL_2_ENCODER_NOT_COMMUTATING,
                                                  WHEEL_3_ENCODER_NOT_COMMUTATING,
                                                  WHEEL_0_HALL_SENSOR_STUCK_LOW,
                                                  WHEEL_1_HALL_SENSOR_STUCK_LOW,
                                                  WHEEL_2_HALL_SENSOR_STUCK_LOW,
                                                  WHEEL_3_HALL_SENSOR_STUCK_LOW,
                                                  DRIBBLER_HALL_SENSOR_STUCK_LOW,
                                                  WHEEL_0_HALL_SENSOR_STUCK_HIGH,
                                                  WHEEL_1_HALL_SENSOR_STUCK_HIGH,
                                                  WHEEL_2_HALL_SENSOR_STUCK_HIGH,
                                                  WHEEL_3_HALL_SENSOR_STUCK_HIGH,
                                                  DRIBBLER_HALL_SENSOR_STUCK_HIGH};

    // Mock robot_status
    RobotStatus robot_status = {.robot = 1,
                                robot_messages,
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

    auto robot_msg = convertRobotStatusToTbotsRobotMsg(robot_status);

    EXPECT_EQ(robot_msg->robot_id(), 1);
    EXPECT_EQ(robot_msg->error_code_size(), expected_error_code.size());
    for (int i = 0; i < robot_msg->error_code_size(); i++)
    {
        EXPECT_EQ(robot_msg->error_code(i), expected_error_code[i]);
    }
    EXPECT_EQ(robot_msg->break_beam_status().ball_in_beam(), true);
    EXPECT_EQ(robot_msg->break_beam_status().break_beam_reading(), 2.0);
    EXPECT_EQ(robot_msg->firmware_status().fw_build_id(), 2);
    EXPECT_EQ(robot_msg->dribbler_status().dribbler_rpm(), 3.0);
    EXPECT_EQ(robot_msg->power_status().battery_voltage(), 4.0);
    EXPECT_EQ(robot_msg->power_status().capacitor_voltage(), 5.0);
    EXPECT_EQ(robot_msg->temperature_status().dribbler_temperature(), 6.0);
    EXPECT_EQ(robot_msg->temperature_status().board_temperature(), 7.0);
}

TEST(RobotStatusTest, test_convert_robot_status_tbots_robot_msg_valid_invalid_robot_msg)
{
    // Mock robot_messages with valid and invalid ErrorCode messages
    std::vector<std::string> robot_messages = {
        "wheel 0 motor hot",
        "Ignore me, not an error message",
        "wheel 0 encoder not commutating",
        "Ignore me, not an error message",
        "wheel 0 Hall sensor stuck low",
        "wheel 0 Hall sensor stuck high",
    };

    // Mock expected return ErrorCode enums from robot_messages
    std::vector<ErrorCode> expected_error_code = {
        WHEEL_0_MOTOR_HOT,
        WHEEL_0_ENCODER_NOT_COMMUTATING,
        WHEEL_0_HALL_SENSOR_STUCK_LOW,
        WHEEL_0_HALL_SENSOR_STUCK_HIGH,
    };

    // Mock robot_status
    RobotStatus robot_status = {.robot = 1,
                                robot_messages,
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

    auto robot_msg = convertRobotStatusToTbotsRobotMsg(robot_status);

    EXPECT_EQ(robot_msg->robot_id(), 1);
    EXPECT_EQ(robot_msg->error_code_size(), expected_error_code.size());
    for (int i = 0; i < robot_msg->error_code_size(); i++)
    {
        EXPECT_EQ(robot_msg->error_code(i), expected_error_code[i]);
    }
    EXPECT_EQ(robot_msg->break_beam_status().ball_in_beam(), true);
    EXPECT_EQ(robot_msg->break_beam_status().break_beam_reading(), 2.0);
    EXPECT_EQ(robot_msg->firmware_status().fw_build_id(), 2);
    EXPECT_EQ(robot_msg->dribbler_status().dribbler_rpm(), 3.0);
    EXPECT_EQ(robot_msg->power_status().battery_voltage(), 4.0);
    EXPECT_EQ(robot_msg->power_status().capacitor_voltage(), 5.0);
    EXPECT_EQ(robot_msg->temperature_status().dribbler_temperature(), 6.0);
    EXPECT_EQ(robot_msg->temperature_status().board_temperature(), 7.0);
}

TEST(RobotStatusTest, test_convert_robot_status_tbots_robot_msg_invalid_robot_msg)
{
    // Mock robot_messages with invalid ErrorCode messages
    std::vector<std::string> robot_messages = {
        "Ignore me, not an error message",
        "Ignore me, not an error message",
        "Ignore me, not an error message",
    };

    // Mock robot_status
    RobotStatus robot_status = {.robot = 1,
                                robot_messages,
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

    auto robot_msg = convertRobotStatusToTbotsRobotMsg(robot_status);

    EXPECT_EQ(robot_msg->robot_id(), 1);
    EXPECT_EQ(robot_msg->error_code_size(), 0);
    EXPECT_EQ(robot_msg->break_beam_status().ball_in_beam(), true);
    EXPECT_EQ(robot_msg->break_beam_status().break_beam_reading(), 2.0);
    EXPECT_EQ(robot_msg->firmware_status().fw_build_id(), 2);
    EXPECT_EQ(robot_msg->dribbler_status().dribbler_rpm(), 3.0);
    EXPECT_EQ(robot_msg->power_status().battery_voltage(), 4.0);
    EXPECT_EQ(robot_msg->power_status().capacitor_voltage(), 5.0);
    EXPECT_EQ(robot_msg->temperature_status().dribbler_temperature(), 6.0);
    EXPECT_EQ(robot_msg->temperature_status().board_temperature(), 7.0);
}
