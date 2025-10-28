#include "software/embedded/motor_controller/motor_fault_indicator.h"

MotorFaultIndicator::MotorFaultIndicator() : drive_enabled(true), motor_faults() {}

MotorFaultIndicator::MotorFaultIndicator(
    bool drive_enabled, std::unordered_set<TbotsProto::MotorFault>& motor_faults)
    : drive_enabled(drive_enabled), motor_faults(motor_faults)
{
}
