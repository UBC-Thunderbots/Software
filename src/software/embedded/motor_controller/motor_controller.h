#pragma once

#include "software/embedded/motor_controller/motor_board.h"
#include "software/embedded/motor_controller/motor_controller_status.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"

class MotorController
{
   public:
    virtual MotorControllerStatus earlyPoll() = 0;

    virtual void reset() = 0;

    virtual MotorFaultIndicator checkDriverFault(const MotorIndex& motor) = 0;

    virtual double readThenWriteVelocity(const MotorIndex& motor,
                                         const int& target_velocity) = 0;

    virtual void immediatelyDisable() = 0;
};
