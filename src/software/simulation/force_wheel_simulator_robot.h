#pragma once

#include "software/simulation/simulator_robot.h"

class ForceWheelSimulatorRobot : public SimulatorRobot
{
    protected:
    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    virtual void applyWheelForceFrontLeft(float force_in_newtons)  = 0;
    virtual void applyWheelForceBackLeft(float force_in_newtons)   = 0;
    virtual void applyWheelForceBackRight(float force_in_newtons)  = 0;
    virtual void applyWheelForceFrontRight(float force_in_newtons) = 0;

    /**
     * Sets the motor to coast (spin freely)
     */
    virtual void coastMotorBackLeft()   = 0;
    virtual void coastMotorBackRight()  = 0;
    virtual void coastMotorFrontLeft()  = 0;
    virtual void coastMotorFrontRight() = 0;

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    virtual void brakeMotorBackLeft()   = 0;
    virtual void brakeMotorBackRight()  = 0;
    virtual void brakeMotorFrontLeft()  = 0;
    virtual void brakeMotorFrontRight() = 0;
};
