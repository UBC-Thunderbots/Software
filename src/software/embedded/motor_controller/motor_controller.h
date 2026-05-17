#pragma once

#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/physics/euclidean_to_wheel.h"

class MotorController
{
   public:
    virtual ~MotorController() = default;

    virtual void setup() = 0;

    virtual void reset() = 0;

    virtual const MotorFaultIndicator& checkFaults(MotorIndex motor) = 0;

    virtual int readThenWriteVelocity(MotorIndex motor, int target_velocity) = 0;

    virtual void updateEuclideanVelocity(EuclideanSpace_t target_euclidean_velocity) = 0;

    virtual void immediatelyDisable() = 0;
};
