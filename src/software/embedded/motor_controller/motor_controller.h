#pragma once

#include "software/embedded/motor_controller/motor_controller_status.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"

class MotorController
{
   public:
    virtual ~MotorController() = default;

    virtual MotorControllerStatus earlyPoll() = 0;

    virtual void setup() = 0;

    virtual void reset() = 0;

    virtual const MotorFaultIndicator& checkFaults(MotorIndex motor) = 0;

    virtual int readThenWriteVelocity(MotorIndex motor, int target_velocity) = 0;

    virtual void immediatelyDisable() = 0;
};
