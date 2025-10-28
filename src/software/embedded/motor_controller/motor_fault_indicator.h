#pragma once

#include <unordered_set>

#include "proto/robot_status_msg.pb.h"

/**
 * Holds motor fault information for a particular motor and whether any fault has
 * caused the motor to be disabled.
 */
struct MotorFaultIndicator
{
    bool drive_enabled;
    std::unordered_set<TbotsProto::MotorFault> motor_faults;

    /**
     * Construct a default indicator of no faults and running motors.
     */
    MotorFaultIndicator();

    /**
     * Construct an indicator with faults and whether the motor is enabled.
     *
     * @param drive_enabled true if the motor is enabled, false if disabled due to a
     * motor fault
     * @param motor_faults  a set of faults associated with this motor
     */
    MotorFaultIndicator(bool drive_enabled,
                        std::unordered_set<TbotsProto::MotorFault>& motor_faults);
};
