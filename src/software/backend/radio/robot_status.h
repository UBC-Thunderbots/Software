#pragma once

#include <string>
#include <vector>

#include "proto/robot_status_msg.pb.h"

// This struct contains various robot diagnostics (e.g. voltages, link quality, etc.)
// and messages related to robot and dongle condition.
struct RadioRobotStatus
{
    // The robot number.
    int32_t robot;

    // Messages related to the robot that are to be displayed in the full_system
    // (usually errors that need attention)
    std::vector<std::string> robot_messages;

    // Messages that are related to the dongle itself
    // (usually errors that need attention)
    std::vector<std::string> dongle_messages;

    // The maximum possible kick speed, in m/s.
    double kick_speed_max;

    // The kick resolution for HScale.
    double kick_speed_resolution;

    // The maximum possible chip distance, in m.
    double chip_distance_max;

    // The chip resolution for HScale.
    double chip_distance_resolution;

    // The maximum power level understood by the direct_dribbler function.
    uint32_t direct_dribbler_max;

    // Whether or not the robot is currently responding to radio communication.
    bool alive;

    // Whether the robot is in direct mode.
    bool direct_control;

    // Whether or not the ball is interrupting the robot’s laser beam.
    bool ball_in_beam;

    // Whether or not the robot’s capacitor is charged enough to kick the ball.
    bool capacitor_charged_enough_to_kick_ball;

    // Indicates when autokick has been fired.
    bool autokick_fired;

    // The voltage on the robot’s battery, in volts.
    double battery_voltage;

    // voltage on the robot’s kicking capacitor, in volts.
    double capacitor_voltage;

    // The reading of the robot’s laser sensor.
    double break_beam_reading;

    // The maximum full-scale deflection (maximum possible reading) of the laser sensor.
    double break_beam_scale;

    // The temperature of the robot’s dribbler motor, in degrees Celsius.
    double dribbler_temperature;

    // The speed of the robot’s dribbler motor, in revolutions per minute.
    int32_t dribbler_speed;

    // The temperature of the robot’s mainboard, in degrees Celsius.
    double board_temperature;

    // The link quality of the last received packet, from 0 (worst) to 1 (best).
    double link_quality;

    // The received signal strength of the last received packet, in decibels.
    int32_t received_signal_strength_db;

    // Whether or not the build ID information is valid.
    bool build_ids_valid;

    // The microcontroller firmware build ID.
    uint32_t fw_build_id;

    // The FPGA bitstream build ID.
    uint32_t fpga_build_id;
};

/**
 * Converts RadioRobotStatus to TbotsProto::RobotStatus
 * Does not convert ChipperKickerStatus,
 * DriveUnitStatus, and NetworkStatus due to insufficient info
 *
 * @param robot_status The RadioRobotStatus
 *
 * @return the equivalent RobotStatus protobuf
 */
std::unique_ptr<TbotsProto::RobotStatus> convertRobotStatusToRobotStatusProto(
    const RadioRobotStatus &robot_status);
