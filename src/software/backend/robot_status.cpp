#include "software/backend/robot_status.h"

#include <map>
#include <string>
#include <vector>

#include "software/logger/logger.h"

std::unique_ptr<TbotsRobotMsg> convertRobotStatusToTbotsRobotMsg(
    const RobotStatus& robot_status)
{
    // Insufficient information to make the TbotsRobotMsg fields for
    // ChipperKickerStatus, DriveUnits, and NetworkStatus
    auto robot_msg = std::make_unique<TbotsRobotMsg>();

    robot_msg->set_robot_id(robot_status.robot);

    ErrorCode error_code;
    for (auto& msg : robot_status.robot_messages)
    {
        // Parse error message into enum string representation
        // by capitalizing and replacing spaces with underscores
        // Ex. "wheel 0 motor hot" -> "WHEEL_0_MOTOR_HOT"
        std::string enum_string = msg;
        std::transform(enum_string.begin(), enum_string.end(), enum_string.begin(),
                       ::toupper);
        std::replace(enum_string.begin(), enum_string.end(), ' ', '_');

        // Map error message string to corresponding enum
        bool errorCodeSuccess = ErrorCode_Parse(enum_string, &error_code);
        // If it is a valid error message, add to error_code
        if (errorCodeSuccess)
        {
            robot_msg->add_error_code(error_code);
        }
        else
        {
            LOG(WARNING) << "'" << msg << "' is not a verified ErrorCode." << std::endl;
        }
    }

    auto break_beam_msg = std::make_unique<BreakBeamStatus>();
    break_beam_msg->set_ball_in_beam(robot_status.ball_in_beam);
    break_beam_msg->set_break_beam_reading(
        static_cast<float>(robot_status.break_beam_reading));
    robot_msg->set_allocated_break_beam_status(break_beam_msg.release());

    auto firmware_status_msg = std::make_unique<FirmwareStatus>();
    firmware_status_msg->set_fw_build_id(robot_status.fw_build_id);
    robot_msg->set_allocated_firmware_status(firmware_status_msg.release());

    auto dribbler_status_msg = std::make_unique<DribblerStatus>();
    dribbler_status_msg->set_dribbler_rpm(
        static_cast<float>(robot_status.dribbler_speed));
    robot_msg->set_allocated_dribbler_status(dribbler_status_msg.release());

    auto power_status_msg = std::make_unique<PowerStatus>();
    power_status_msg->set_battery_voltage(
        static_cast<float>(robot_status.battery_voltage));
    power_status_msg->set_capacitor_voltage(
        static_cast<float>(robot_status.capacitor_voltage));
    robot_msg->set_allocated_power_status(power_status_msg.release());

    auto temperature_status_msg = std::make_unique<TemperatureStatus>();
    temperature_status_msg->set_dribbler_temperature(
        static_cast<float>(robot_status.dribbler_temperature));
    temperature_status_msg->set_board_temperature(
        static_cast<float>(robot_status.board_temperature));
    robot_msg->set_allocated_temperature_status(temperature_status_msg.release());

    return robot_msg;
}
