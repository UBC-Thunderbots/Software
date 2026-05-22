#pragma once

#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/physics/euclidean_to_wheel.h"

/**
 * Abstract interface for motor controllers.
 *
 * Implementations provide hardware-specific setup, fault reporting,
 * and velocity control for the robot's motors.
 */
class MotorController
{
   public:
    virtual ~MotorController() = default;

    /**
     * Initializes the motor controller hardware and any internal state.
     */
    virtual void setup() = 0;

    /**
     * Resets the underlying motor controller hardware.
     */
    virtual void reset() = 0;

    /**
     * Returns the current fault state for the given motor.
     *
     * @param motor the motor to inspect
     * @return a MotorFaultIndicator indicating the faults for that motor
     */
    virtual const MotorFaultIndicator& checkFaults(MotorIndex motor) = 0;

    /**
     * Reads the current velocity and writes a new target velocity for a motor.
     *
     * @param motor the motor to command
     * @param target_velocity the desired target velocity for the motor in mechanical RPM
     * @return the current measured velocity of the motor in mechanical RPM
     */
    virtual int readThenWriteVelocity(MotorIndex motor, int target_velocity) = 0;

    /**
     * Updates the motor controller with the target local velocity of the robot
     * in Euclidean space.
     *
     * @param target_euclidean_velocity the target local velocity of the robot
     */
    virtual void updateEuclideanVelocity(EuclideanSpace_t target_euclidean_velocity) = 0;

    /**
     * Immediately disables all motors.
     */
    virtual void immediatelyDisable() = 0;
};