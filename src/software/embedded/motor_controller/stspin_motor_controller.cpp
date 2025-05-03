#include "software/embedded/motor_controller/stspin_motor_controller.h"

MotorControllerStatus StSpinMotorController::earlyPoll()
{
    return MotorControllerStatus::OK;
}

void StSpinMotorController::reset() {}

MotorFaultIndicator StSpinMotorController::checkDriverFault(const MotorIndex& motor)
{
    return MotorFaultIndicator();
}

double StSpinMotorController::readThenWriteVelocity(const MotorIndex& motor,
                                                    const int& target_velocity)
{
    return 0.0;
}

void StSpinMotorController::immediatelyDisable() {}
