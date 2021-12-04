#include "software/backend/radio/robot_status.h"

#include <map>
#include <string>
#include <vector>

#include "software/backend/radio/mrf/messages.h"
#include "software/logger/logger.h"

static const std::map<const std::string, TbotsProto::ErrorCode>
    dongle_message_error_codes = {
        {MRF::ROBOT_DEAD_MESSAGE, TbotsProto::ErrorCode::ROBOT_DEAD},
        {MRF::ESTOP_BROKEN_MESSAGE, TbotsProto::ErrorCode::ESTOP_BROKEN},
        {MRF::RX_FCS_FAIL_MESSAGE, TbotsProto::ErrorCode::RX_FCS_FAIL},
        {MRF::SECOND_DONGLE_MESSAGE, TbotsProto::ErrorCode::SECOND_DONGLE},
        {MRF::TRANSMIT_QUEUE_FULL_MESSAGE, TbotsProto::ErrorCode::TRANSMIT_QUEUE_FULL},
        {MRF::RECEIVE_QUEUE_FULL_MESSAGE, TbotsProto::ErrorCode::RECEIVE_QUEUE_FULL},
        {MRF::PACKET_ABOVE_MAX_SIZE, TbotsProto::ErrorCode::PACKET_ABOVE_MAX_SIZE}};

std::unique_ptr<TbotsProto::RobotStatus> convertRobotStatusToRobotStatusProto(
    const RadioRobotStatus& robot_status)
{
    // Insufficient information to make the RobotStatus fields for
    // ChipperKickerStatus, DriveUnitStatus, and NetworkStatus
    auto robot_msg = std::make_unique<TbotsProto::RobotStatus>();

    robot_msg->set_robot_id(robot_status.robot);

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
        TbotsProto::ErrorCode error_code;
        bool errorCodeSuccess = TbotsProto::ErrorCode_Parse(enum_string, &error_code);
        // If it is a valid error message, add to error_code
        if (errorCodeSuccess)
        {
            robot_msg->add_error_code(error_code);
        }
        else
        {
            LOG(WARNING) << "'" << msg << "' is not a valid TbotsProto::ErrorCode."
                         << std::endl;
        }
    }

    for (auto& msg : robot_status.dongle_messages)
    {
        auto error_code_it = dongle_message_error_codes.find(msg);
        if (error_code_it != dongle_message_error_codes.end())
        {
            robot_msg->add_error_code(error_code_it->second);
        }
        else
        {
            LOG(WARNING) << "'" << msg << "' is not a valid TbotsProto::ErrorCode."
                         << std::endl;
        }
    }

    auto break_beam_msg = std::make_unique<TbotsProto::BreakBeamStatus>();
    break_beam_msg->set_ball_in_beam(robot_status.ball_in_beam);
    break_beam_msg->set_break_beam_reading(
        static_cast<float>(robot_status.break_beam_reading));
    *(robot_msg->mutable_break_beam_status()) = *break_beam_msg;

    auto firmware_status_msg = std::make_unique<TbotsProto::FirmwareStatus>();
    firmware_status_msg->set_fw_build_id(robot_status.fw_build_id);
    *(robot_msg->mutable_firmware_status()) = *firmware_status_msg;

    auto dribbler_status_msg = std::make_unique<TbotsProto::DribblerStatus>();
    dribbler_status_msg->set_dribbler_rpm(
        static_cast<float>(robot_status.dribbler_speed));
    *(robot_msg->mutable_dribbler_status()) = *dribbler_status_msg;

    auto power_status_msg = std::make_unique<TbotsProto::PowerStatus>();
    power_status_msg->set_battery_voltage(
        static_cast<float>(robot_status.battery_voltage));
    power_status_msg->set_capacitor_voltage(
        static_cast<float>(robot_status.capacitor_voltage));
    *(robot_msg->mutable_power_status()) = *power_status_msg;

    auto temperature_status_msg = std::make_unique<TbotsProto::TemperatureStatus>();
    temperature_status_msg->set_dribbler_temperature(
        static_cast<float>(robot_status.dribbler_temperature));
    temperature_status_msg->set_board_temperature(
        static_cast<float>(robot_status.board_temperature));
    *(robot_msg->mutable_temperature_status()) = *temperature_status_msg;

    return robot_msg;
}
