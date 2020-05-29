#include "software/backend/robot_status.h"

TbotsRobotMsg convertRobotStatusToTbotsRobotMsg(RobotStatus* robot_status)
{
    // Insufficient ErrorCode, ChipperKickerStatus, DriveUnits, and NetworkStatus info
    TbotsRobotMsg robot_msg;

    robot_msg.set_robot_id(robot_status->robot);

    auto break_beam_msg = std::make_unique<BreakBeamStatus>();
    break_beam_msg->set_ball_in_beam(robot_status->ball_in_beam);
    break_beam_msg->set_break_beam_reading(
        static_cast<float>(robot_status->break_beam_reading));
    robot_msg.set_allocated_break_beam_status(break_beam_msg.release());

    auto firmware_status_msg = std::make_unique<FirmwareStatus>();
    firmware_status_msg->set_fw_build_id(robot_status->fw_build_id);
    robot_msg.set_allocated_firmware_status(firmware_status_msg.release());

    auto dribbler_status_msg = std::make_unique<DribblerStatus>();
    dribbler_status_msg->set_dribbler_rpm(
        static_cast<float>(robot_status->dribbler_speed));
    robot_msg.set_allocated_dribbler_status(dribbler_status_msg.release());

    auto power_status_msg = std::make_unique<PowerStatus>();
    power_status_msg->set_battery_voltage(
        static_cast<float>(robot_status->battery_voltage));
    power_status_msg->set_capacitor_voltage(
        static_cast<float>(robot_status->capacitor_voltage));
    robot_msg.set_allocated_power_status(power_status_msg.release());

    auto temperature_status_msg = std::make_unique<TemperatureStatus>();
    temperature_status_msg->set_dribbler_temperature(
        (float)robot_status->dribbler_temperature);
    temperature_status_msg->set_board_temperature(
        static_cast<float>(robot_status->board_temperature));
    robot_msg.set_allocated_temperature_status(temperature_status_msg.release());

    return robot_msg;
}
