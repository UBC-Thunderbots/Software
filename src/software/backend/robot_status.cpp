#include "software/backend/robot_status.h"

#include <map>
#include <string>
#include <vector>

#include "software/logger/logger.h"

TbotsRobotMsg convertRobotStatusToTbotsRobotMsg(const RobotStatus& robot_status)
{
    std::map<std::string, ErrorCode> errorStringToEnumMap;

    // Fill map with ERROR_LT_MESSAGES according to messages.h
    for (int ErrorCodeInt = CHARGE_TIMEOUT;
         ErrorCodeInt != DRIBBLER_HALL_SENSOR_STUCK_HIGH; ErrorCodeInt++)
    {
        std::string ErrorString =
            ErrorCode_descriptor()
                ->FindValueByNumber(static_cast<ErrorCode>(ErrorCodeInt))
                ->name();
        errorStringToEnumMap.insert(std::pair<std::string, ErrorCode>(
            ErrorString, static_cast<ErrorCode>(ErrorCodeInt)));
    }

    // Insufficient information to make the TbotsRobotMsg fields for
    // ChipperKickerStatus, DriveUnits, and NetworkStatus
    TbotsRobotMsg robot_msg;

    robot_msg.set_robot_id(robot_status.robot);

    for (const auto& msg : robot_status.robot_messages)
    {
        // If it is a valid error message, add to error_code
        if (errorStringToEnumMap.find(msg) != errorStringToEnumMap.end())
        {
            robot_msg.add_error_code(errorStringToEnumMap[msg]);
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
    robot_msg.set_allocated_break_beam_status(break_beam_msg.release());

    auto firmware_status_msg = std::make_unique<FirmwareStatus>();
    firmware_status_msg->set_fw_build_id(robot_status.fw_build_id);
    robot_msg.set_allocated_firmware_status(firmware_status_msg.release());

    auto dribbler_status_msg = std::make_unique<DribblerStatus>();
    dribbler_status_msg->set_dribbler_rpm(
        static_cast<float>(robot_status.dribbler_speed));
    robot_msg.set_allocated_dribbler_status(dribbler_status_msg.release());

    auto power_status_msg = std::make_unique<PowerStatus>();
    power_status_msg->set_battery_voltage(
        static_cast<float>(robot_status.battery_voltage));
    power_status_msg->set_capacitor_voltage(
        static_cast<float>(robot_status.capacitor_voltage));
    robot_msg.set_allocated_power_status(power_status_msg.release());

    auto temperature_status_msg = std::make_unique<TemperatureStatus>();
    temperature_status_msg->set_dribbler_temperature(
        static_cast<float>(robot_status.dribbler_temperature));
    temperature_status_msg->set_board_temperature(
        static_cast<float>(robot_status.board_temperature));
    robot_msg.set_allocated_temperature_status(temperature_status_msg.release());

    return robot_msg;
}
