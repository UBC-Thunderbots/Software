#include "software/embedded/motor_controller/motor_fault_indicator.h"

MotorFaultIndicator::MotorFaultIndicator() : drive_enabled(true) {}

MotorFaultIndicator::MotorFaultIndicator(
    const bool drive_enabled, const std::unordered_set<TbotsProto::MotorFault>& motor_faults)
    : drive_enabled(drive_enabled), faults(motor_faults)
{
}

bool MotorFaultIndicator::requiresReset() const
{
    return !drive_enabled || faults.find(TbotsProto::MotorFault::RESET) != faults.end();
}
