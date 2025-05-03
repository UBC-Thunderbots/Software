#pragma once

#include "software/embedded/motor_controller/motor_controller.h"

class StSpinMotorController : public MotorController
{
   public:
    MotorControllerStatus earlyPoll() override;

    void reset() override;

    MotorFaultIndicator checkDriverFault(const MotorIndex& motor) override;

    double readThenWriteVelocity(const MotorIndex& motor,
                                 const int& target_velocity) override;

    void immediatelyDisable() override;
};
